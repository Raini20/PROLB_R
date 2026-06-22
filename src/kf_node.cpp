#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "probabilistic_robot_lab/anchor.hpp"
#include "probabilistic_robot_lab/detection_markers.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*
 * Kalman Filter Node — Odometry + Landmark Correction
 * =====================================================
 * State:         mu = [x, y, theta]^T
 * Control input: u  = [v, omega]^T     (from /cmd_vel, robot frame)
 *
 * Prediction  (Thrun Table 3.1, Lines 2–3)
 *   mu_bar    = A * mu + B(theta) * u
 *   Sigma_bar = A * Sigma * A^T + R
 *   NOTE: B is state-dependent — linearisation approximation.
 *
 * Correction 1 — Odometry  (Lines 4–6)
 *   z = [x, y, theta],  C = I_3
 *   Timestamp check: skipped if odom is stale (> 500 ms).
 *
 * Correction 2 — Landmarks  (1-D range per landmark)
 *   C_lm = dh/dmu — linearised Jacobian of range function.
 *
 * Tunable via ROS2 parameters (mandatory noise experiments):
 *   sigma_process  (default 0.01)  scales R = sigma^2 * I
 *   sigma_meas     (default 0.10)  scales Q = sigma^2 * I
 *
 * ref: Thrun (2006) Table 3.1
 */


static inline double wrapAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kf_node"), odom_received_(false)
    {
        this->declare_parameter("sigma_process", 0.01);
        this->declare_parameter("sigma_meas",    0.10);

        double sp = this->get_parameter("sigma_process").as_double();
        double sm = this->get_parameter("sigma_meas").as_double();

        mu_    = Eigen::Vector3d::Zero();
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;
        A_     = Eigen::Matrix3d::Identity();
        R_     = Eigen::Matrix3d::Identity() * (sp * sp);
        C_     = Eigen::Matrix3d::Identity();
        Q_     = Eigen::Matrix3d::Identity() * (sm * sm);
        q_lm_  = LM_SIGMA_R * LM_SIGMA_R;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&KalmanFilterNode::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&KalmanFilterNode::scanCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_kf", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/kf_markers", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(),
            "KF Node started  sigma_process=%.4f  sigma_meas=%.4f", sp, sm);
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

        // RANSAC wall lines: extraction (pose-independent) + association.
        auto lines = extractWalls(msg);
        walls_     = associateWalls(lines, mu_(0), mu_(1), mu_(2));

        marker_pub_->publish(buildDetectionMarkers(
            mu_(0), mu_(1), detected_, walls_, "kf",
            0.12f, 0.47f, 0.71f, this->get_clock()->now()));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        double v = msg->linear.x, omega = msg->angular.z, theta = mu_(2);

        // PREDICTION
        Eigen::Matrix<double, 3, 2> B;
        B << std::cos(theta)*dt, 0.0,
             std::sin(theta)*dt, 0.0,
             0.0,                dt;
        Eigen::Vector3d mu_bar    = A_ * mu_ + B * Eigen::Vector2d(v, omega);
        Eigen::Matrix3d Sigma_bar = A_ * Sigma_ * A_.transpose() + R_;

        // CORRECTION 1: Odometry (with timestamp check)
        bool odom_valid = odom_received_ &&
            (this->get_clock()->now() - last_odom_time_).seconds() < 0.5;

        if (odom_valid) {
            Eigen::Matrix3d S = C_ * Sigma_bar * C_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * C_.transpose() * S.inverse();
            Eigen::Vector3d innov = z_ - C_ * mu_bar;
            innov(2) = wrapAngle(innov(2));
            mu_    = mu_bar + K * innov;
            Sigma_ = (Eigen::Matrix3d::Identity() - K * C_) * Sigma_bar;
        } else {
            mu_ = mu_bar;  Sigma_ = Sigma_bar;
        }

        // CORRECTION 2 + 3: Combined anchor feature (LM_ML pillar + corner walls)
        // -----------------------------------------------------------------------
        // See ekf_node.cpp for rationale.  Wall measurement model is LINEAR
        // in the pose, so the KF update here is exact (no linearization needed).
        if (anchor_detected(detected_, walls_)) {

            // Landmark range correction
            for (const auto& obs : detected_) {
                double dx = obs.mx - mu_(0), dy = obs.my - mu_(1);
                double r2 = dx*dx + dy*dy, r = std::sqrt(r2);
                if (r < 1e-4) continue;
                Eigen::Matrix<double,1,3> C_lm;
                C_lm << -dx/r, -dy/r, 0.0;
                double innov = obs.range - r;
                double S_lm  = (C_lm * Sigma_ * C_lm.transpose())(0,0) + q_lm_;
                Eigen::Vector3d K_lm = Sigma_ * C_lm.transpose() / S_lm;
                mu_    += K_lm * innov;
                mu_(2)  = wrapAngle(mu_(2));
                Sigma_  = (Eigen::Matrix3d::Identity() - K_lm * C_lm) * Sigma_;
            }

            // Wall correction (exact linear model)
            for (const auto& w : walls_) {
                const double ca = std::cos(w.aw), sa = std::sin(w.aw);

                Eigen::Vector2d z(w.a_meas, w.r_meas);
                Eigen::Vector2d zhat(w.aw - mu_(2),
                                     w.rw - (mu_(0) * ca + mu_(1) * sa));
                Eigen::Vector2d innov = z - zhat;
                innov(0) = wrapAngle(innov(0));

                Eigen::Matrix<double,2,3> C_w;
                C_w <<   0.0, 0.0, -1.0,
                         -ca,  -sa,  0.0;

                Eigen::Matrix2d R_w = Eigen::Matrix2d::Zero();
                R_w(0,0) = WALL_SIGMA_A * WALL_SIGMA_A;
                R_w(1,1) = WALL_SIGMA_R * WALL_SIGMA_R;

                Eigen::Matrix2d S_w = C_w * Sigma_ * C_w.transpose() + R_w;
                Eigen::Matrix<double,3,2> K_w = Sigma_ * C_w.transpose() * S_w.inverse();
                mu_    += K_w * innov;
                mu_(2)  = wrapAngle(mu_(2));
                Sigma_  = (Eigen::Matrix3d::Identity() - K_w * C_w) * Sigma_;
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
    Eigen::Matrix3d Sigma_, A_, R_, C_, Q_;
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
