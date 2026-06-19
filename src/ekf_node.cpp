#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "probabilistic_robot_lab/landmark_map.hpp"
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

struct LandmarkObs { double mx, my, range; };

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
        detected_.clear();
        for (const auto& lm : LANDMARK_MAP) {
            double dx    = lm.x - mu_(0);
            double dy    = lm.y - mu_(1);
            double r_exp = std::sqrt(dx*dx + dy*dy);
            if (r_exp < msg->range_min + 0.05 || r_exp > msg->range_max - 0.05) continue;
            double bearing = wrapAngle(std::atan2(dy, dx) - mu_(2));
            if (bearing < msg->angle_min || bearing > msg->angle_max) continue;
            int idx = static_cast<int>(
                std::round((bearing - msg->angle_min) / msg->angle_increment));
            if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) continue;
            float r_meas = msg->ranges[idx];
            if (!std::isfinite(r_meas)) continue;
            if (r_meas < msg->range_min || r_meas > msg->range_max) continue;
            if (std::abs(r_meas - r_exp) > LM_GATE_M) continue;
            detected_.push_back({lm.x, lm.y, static_cast<double>(r_meas)});
        }
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

        // CORRECTION 2: Landmarks
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
        msg.pose.covariance[7]  = Sigma_(1,1);
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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
