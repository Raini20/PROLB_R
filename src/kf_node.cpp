#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <cmath>
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*
 * Kalman Filter Node
 * State:         mu = [x, y, theta]^T
 * Control input: u  = [v_x, omega]^T  (from /cmd_vel, Robot Frame)
 * Measurement:   z  = [x, y, theta]^T (from /odom, World Frame)
 *
 * Algorithm Kalman_filter(mu_{t-1}, Sigma_{t-1}, u_t, z_t)
 * ref: Thrun (2006), Probabilistic Robotics, Table 3.1
 *
 * NOTE: B_t is state-dependent because u_t is in Robot Frame.
 *       Transformation to World Frame requires cos(theta)/sin(theta).
 *       This is a linearization — the EKF handles this properly via Jacobian.
 *
 * Thrun notation:
 *   R_t = process noise covariance  (prediction, Line 3)
 *   Q_t = measurement noise covariance (correction, Line 4)
 *   C_t = measurement matrix = Identity (z maps directly to state)
 */

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kf_node"), odom_received_(false)
    {
        // --- Initial State ---
        // mu_{t-1}: initial state [x, y, theta]
        mu_ = Eigen::Vector3d::Zero();

        // Sigma_{t-1}: initial covariance
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;

        // --- System Matrices ---
        // A_t: state transition (identity — no velocity in state vector)
        // Line 2: mu_bar = A * mu_{t-1} + B * u_t
        A_ = Eigen::Matrix3d::Identity();

        // R_t: process noise covariance (Thrun notation, Line 3)
        // Line 3: Sigma_bar = A * Sigma_{t-1} * A^T + R_t
        R_ = Eigen::Matrix3d::Identity() * 0.01;

        // C_t: measurement matrix — maps state to measurement space
        // z = [x, y, theta] = C * mu  =>  C = Identity
        // Line 4: K_t = Sigma_bar * C^T * (C * Sigma_bar * C^T + Q_t)^{-1}
        C_ = Eigen::Matrix3d::Identity();

        // Q_t: measurement noise covariance (Thrun notation, Line 4)
        Q_ = Eigen::Matrix3d::Identity() * 0.1;

        // --- Subscribers ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&KalmanFilterNode::odomCallback, this, std::placeholders::_1));

        // --- Publisher ---
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_kf", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "KF Node started");
    }

private:
    // Cache latest odom measurement
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract z_t = [x, y, theta] from odometry
        z_(0) = msg->pose.pose.position.x;
        z_(1) = msg->pose.pose.position.y;

        // Convert quaternion to theta (rotation around z-axis)
        // theta = 2 * atan2(q.z, q.w)
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        z_(2) = 2.0 * std::atan2(qz, qw);

        odom_received_ = true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 1.0) return;  // skip bad dt

        // u_t = [v_x, omega]^T from /cmd_vel (Robot Frame)
        double vx    = msg->linear.x;
        double omega = msg->angular.z;
        Eigen::Vector2d u(vx, omega);

        // B_t: control input matrix (3x2), state-dependent via theta
        // Transforms u_t from Robot Frame to World Frame
        double theta = mu_(2);
        Eigen::Matrix<double, 3, 2> B;
        B << std::cos(theta) * dt,  0.0,
             std::sin(theta) * dt,  0.0,
             0.0,                   dt;

        // -------------------------------------------------------
        // PREDICTION STEP
        // -------------------------------------------------------
        // Line 2 (Thrun): mu_bar_t = A_t * mu_{t-1} + B_t * u_t
        Eigen::Vector3d mu_bar = A_ * mu_ + B * u;

        // Line 3 (Thrun): Sigma_bar_t = A_t * Sigma_{t-1} * A_t^T + R_t
        Eigen::Matrix3d Sigma_bar = A_ * Sigma_ * A_.transpose() + R_;

        // -------------------------------------------------------
        // CORRECTION STEP  (only if we have a measurement)
        // -------------------------------------------------------
        if (odom_received_)
        {
            // Line 4 (Thrun): K_t = Sigma_bar * C^T * (C * Sigma_bar * C^T + Q_t)^{-1}
            Eigen::Matrix3d S = C_ * Sigma_bar * C_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * C_.transpose() * S.inverse();

            // Line 5 (Thrun): mu_t = mu_bar + K_t * (z_t - C_t * mu_bar)
            Eigen::Vector3d innovation = z_ - C_ * mu_bar;

            // Wrap theta innovation to [-pi, pi]
            while (innovation(2) >  M_PI) innovation(2) -= 2.0 * M_PI;
            while (innovation(2) < -M_PI) innovation(2) += 2.0 * M_PI;

            mu_ = mu_bar + K * innovation;

            // Line 6 (Thrun): Sigma_t = (I - K_t * C_t) * Sigma_bar
            Sigma_ = (Eigen::Matrix3d::Identity() - K * C_) * Sigma_bar;
        }
        else
        {
            // No measurement yet — prediction only
            mu_    = mu_bar;
            Sigma_ = Sigma_bar;
        }

        publishPose();
    }

    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        auto now = this->get_clock()->now();
        msg.header.stamp.sec     = (int32_t)(now.nanoseconds() / 1000000000LL);
        msg.header.stamp.nanosec = (uint32_t)(now.nanoseconds() % 1000000000LL);
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = mu_(0);
        msg.pose.pose.position.y = mu_(1);
        msg.pose.pose.orientation.z = std::sin(mu_(2) / 2.0);
        msg.pose.pose.orientation.w = std::cos(mu_(2) / 2.0);

        // Covariance (6x6 row-major: x,y,z,rx,ry,rz)
        // We track x,y,theta → indices [0,0], [1,1], [5,5]
        msg.pose.covariance[0]  = Sigma_(0, 0);  // x variance
        msg.pose.covariance[7]  = Sigma_(1, 1);  // y variance
        msg.pose.covariance[35] = Sigma_(2, 2);  // theta variance

        pub_->publish(msg);
    }

    // State
    Eigen::Vector3d mu_;       // [x, y, theta]
    Eigen::Matrix3d Sigma_;    // Covariance
    Eigen::Vector3d z_;        // Latest measurement from /odom
    bool odom_received_;

    // System matrices (Thrun notation)
    Eigen::Matrix3d A_;   // State transition
    Eigen::Matrix3d R_;   // Process noise covariance
    Eigen::Matrix3d C_;   // Measurement matrix
    Eigen::Matrix3d Q_;   // Measurement noise covariance

    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
