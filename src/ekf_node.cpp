#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "probabilistic_robot_lab/anchor.hpp"
#include "probabilistic_robot_lab/detection_markers.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*
 * Extended Kalman Filter Node — Odometry + Landmark Correction
 * =============================================================
 * State:         mu = [x, y, theta]^T
 * Control input: u  = [v, omega]^T     (from /cmd_vel, robot frame)
 *
 * Prediction  (Thrun Table 3.3, Lines 1–3)
 *   mu_bar    = g(u, mu)               — exact nonlinear motion model
 *   Sigma_bar = G * Sigma * G^T + R   — G = dg/dmu (Jacobian)
 *
 * Correction 1 — Odometry  (Lines 4–6, H = I_3)
 *   Timestamp check: skipped if odom is stale (> 500 ms).
 *
 * Correction 2 — Landmarks  (Thrun Table 7.2, adapted)
 *   H_j = dh/dmu — 1×3 Jacobian of range function
 *
 * Tunable via ROS2 parameters (mandatory noise experiments):
 *   sigma_process  (default 0.01)  scales R = sigma^2 * I
 *   sigma_meas     (default 0.10)  scales Q = sigma^2 * I
 *
 * ref: Thrun (2006) Table 3.3 + Table 7.2
 */


static inline double wrapAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("ekf_node"), odom_received_(false)
    {
        this->declare_parameter("sigma_process", 0.01);
        this->declare_parameter("sigma_meas",    0.10);

        double sp = this->get_parameter("sigma_process").as_double();
        double sm = this->get_parameter("sigma_meas").as_double();

        mu_    = Eigen::Vector3d::Zero();
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;
        R_     = Eigen::Matrix3d::Identity() * (sp * sp);
        H_     = Eigen::Matrix3d::Identity();
        Q_     = Eigen::Matrix3d::Identity() * (sm * sm);
        q_lm_  = LM_SIGMA_R * LM_SIGMA_R;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ExtendedKalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ExtendedKalmanFilterNode::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ExtendedKalmanFilterNode::scanCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_ekf", 10);
        lm_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/ekf_landmark_count", 10);
        wall_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/ekf_wall_count", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ekf_markers", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(),
            "EKF Node started  sigma_process=%.4f  sigma_meas=%.4f", sp, sm);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        z_(0) = msg->pose.pose.position.x;
        z_(1) = msg->pose.pose.position.y;
        z_(2) = 2.0 * std::atan2(
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        last_odom_time_ = this->get_clock()->now();
        odom_received_  = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Stage 1: detect real pillars from the raw scan (no pose used).
        // Stage 2: associate each detection to a known map landmark using
        //          the current estimate mu_ (matching only).
        auto cyl  = detectCylinders(msg);
        detected_ = associateLandmarks(cyl, mu_(0), mu_(1), mu_(2));

        // Stage 1+2 for walls: RANSAC line extraction + association.
        auto lines = extractWalls(msg);
        walls_     = associateWalls(lines, mu_(0), mu_(1), mu_(2));

        for (const auto& w : walls_)
            RCLCPP_INFO(this->get_logger(), "WALL aw=%.2f rw=%.2f | a_meas=%.2f r_meas=%.2f",
                        w.aw, w.rw, w.a_meas, w.r_meas);

        // Publish raw detection counts (stage 1+2, pose-independent up to the
        // small association-pose dependence) for paper plots — lets us shade
        // "feature visible" periods on the error/RMSE/covariance time series.
        // Publish detection counts for the data logger / plot shading.
        // anchor_count > 0 only when the combined feature fires; the
        // separate lm/wall counts are kept for debugging.
        bool anc = anchor_detected(detected_, walls_);
        std_msgs::msg::Int32 lm_msg;
        lm_msg.data = anc ? static_cast<int32_t>(detected_.size()) : 0;
        lm_count_pub_->publish(lm_msg);

        std_msgs::msg::Int32 wall_msg;
        wall_msg.data = anc ? static_cast<int32_t>(walls_.size()) : 0;
        wall_count_pub_->publish(wall_msg);

        marker_pub_->publish(buildDetectionMarkers(
            mu_(0), mu_(1), detected_, walls_, "ekf",
            0.84f, 0.15f, 0.16f, this->get_clock()->now()));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        double v = msg->linear.x, omega = msg->angular.z, theta = mu_(2);

        // PREDICTION (Lines 1–3)
        Eigen::Vector3d mu_bar;
        mu_bar(0) = mu_(0) + v * std::cos(theta) * dt;
        mu_bar(1) = mu_(1) + v * std::sin(theta) * dt;
        mu_bar(2) = mu_(2) + omega * dt;

        Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
        G(0,2) = -v * std::sin(theta) * dt;
        G(1,2) =  v * std::cos(theta) * dt;

        Eigen::Matrix3d Sigma_bar = G * Sigma_ * G.transpose() + R_;

        // CORRECTION 1: Odometry (with timestamp check)
        bool odom_valid = odom_received_ &&
            (this->get_clock()->now() - last_odom_time_).seconds() < 0.5;

        if (odom_valid) {
            Eigen::Matrix3d S = H_ * Sigma_bar * H_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * H_.transpose() * S.inverse();
            Eigen::Vector3d innov = z_ - mu_bar;
            innov(2) = wrapAngle(innov(2));
            mu_    = mu_bar + K * innov;
            Sigma_ = (Eigen::Matrix3d::Identity() - K * H_) * Sigma_bar;
        } else {
            mu_ = mu_bar;  Sigma_ = Sigma_bar;
        }

        // CORRECTION 2 + 3: Combined anchor feature (LM_ML pillar + corner walls)
        // -----------------------------------------------------------------------
        // Neither the pillar alone (any of the 9 identical pillars could match)
        // nor the wall alone (association uses the current pose estimate) is an
        // unambiguous fix.  Together — one cylinder next to two specific wall
        // segments — the feature is unique in the arena and the correction is
        // applied without relying purely on the prior pose estimate for identity.
        if (anchor_detected(detected_, walls_)) {

            // Landmark range correction (one update per detected landmark)
            for (const auto& obs : detected_) {
                double dx = obs.mx - mu_(0), dy = obs.my - mu_(1);
                double r2 = dx*dx + dy*dy, r = std::sqrt(r2);
                if (r < 1e-4) continue;
                Eigen::Matrix<double,1,3> H_j;
                H_j << -dx/r, -dy/r, 0.0;
                double innov = obs.range - r;
                double S_j   = (H_j * Sigma_ * H_j.transpose())(0,0) + q_lm_;
                Eigen::Vector3d K_j = Sigma_ * H_j.transpose() / S_j;
                mu_    += K_j * innov;
                mu_(2)  = wrapAngle(mu_(2));
                Sigma_  = (Eigen::Matrix3d::Identity() - K_j * H_j) * Sigma_;
            }

            // Wall correction — linear model, exact for KF and EKF alike
            for (const auto& w : walls_) {
                const double ca = std::cos(w.aw), sa = std::sin(w.aw);

                Eigen::Vector2d z(w.a_meas, w.r_meas);
                Eigen::Vector2d zhat(w.aw - mu_(2),
                                     w.rw - (mu_(0) * ca + mu_(1) * sa));
                Eigen::Vector2d innov = z - zhat;
                innov(0) = wrapAngle(innov(0));

                Eigen::Matrix<double,2,3> H_w;
                H_w <<   0.0, 0.0, -1.0,
                         -ca,  -sa,  0.0;

                Eigen::Matrix2d R_w = Eigen::Matrix2d::Zero();
                R_w(0,0) = WALL_SIGMA_A * WALL_SIGMA_A;
                R_w(1,1) = WALL_SIGMA_R * WALL_SIGMA_R;

                Eigen::Matrix2d S_w = H_w * Sigma_ * H_w.transpose() + R_w;
                Eigen::Matrix<double,3,2> K_w = Sigma_ * H_w.transpose() * S_w.inverse();
                mu_    += K_w * innov;
                mu_(2)  = wrapAngle(mu_(2));
                Sigma_  = (Eigen::Matrix3d::Identity() - K_w * H_w) * Sigma_;
            }

        } // end anchor gate

        publishPose();
    }

    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        auto now = this->get_clock()->now();
        msg.header.stamp.sec     = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
        msg.header.frame_id = "odom";
        msg.pose.pose.position.x    = mu_(0);
        msg.pose.pose.position.y    = mu_(1);
        msg.pose.pose.orientation.z = std::sin(mu_(2) / 2.0);
        msg.pose.pose.orientation.w = std::cos(mu_(2) / 2.0);
        msg.pose.covariance[0]  = Sigma_(0,0);
        msg.pose.covariance[1]  = Sigma_(0,1);
        msg.pose.covariance[5]  = Sigma_(0,2);
        msg.pose.covariance[6]  = Sigma_(1,0);
        msg.pose.covariance[7]  = Sigma_(1,1);
        msg.pose.covariance[11] = Sigma_(1,2);
        msg.pose.covariance[30] = Sigma_(2,0);
        msg.pose.covariance[31] = Sigma_(2,1);
        msg.pose.covariance[35] = Sigma_(2,2);
        pub_->publish(msg);
    }

    Eigen::Vector3d mu_;
    Eigen::Matrix3d Sigma_, R_, H_, Q_;
    Eigen::Vector3d z_;
    double          q_lm_;
    bool            odom_received_;
    rclcpp::Time    last_odom_time_, last_time_;
    std::vector<LandmarkObs> detected_;
    std::vector<WallObs>     walls_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lm_count_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wall_count_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
