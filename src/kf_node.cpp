#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>
#include <cmath>
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*
 * Kalman Filter Node
 * State:         x = [x, y, theta]^T
 * Control input: u = [v_x, omega]^T  (from /cmd_vel, Robot Frame)
 *
 * Algorithm Kalman_filter(mu_{t-1}, Sigma_{t-1}, u_t, z_t)
 * ref: Thrun (2006), Probabilistic Robotics, Table 3.1
 *
 * NOTE: B_t is state-dependent because u_t is in Robot Frame.
 *       Transformation to World Frame requires cos(theta)/sin(theta).
 *       This is a linearization — the EKF handles this properly via Jacobian.
 *
 * Thrun notation used:
 *   R_t = process noise covariance  (prediction step)
 *   Q_t = measurement noise covariance (correction step)
 */

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kf_node")
    {
        // mu_{t-1}: initial state estimate [x, y, theta]
        mu_ = Eigen::Vector3d::Zero();

        // Sigma_{t-1}: initial covariance
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;

        // A_t: state transition matrix (identity)
        // Line 2: mu_bar_t = A_t * mu_{t-1} + B_t * u_t
        A_ = Eigen::Matrix3d::Identity();

        // R_t: process noise covariance (Thrun notation)
        // Line 3: Sigma_bar_t = A_t * Sigma_{t-1} * A_t^T + R_t
        R_ = Eigen::Matrix3d::Identity() * 0.01;

        sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10,
            std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_kf", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "KF Node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // u_t = [v_x, omega]^T from /cmd_vel (Robot Frame)
        double vx    = msg->twist.linear.x;
        double omega = msg->twist.angular.z;
        Eigen::Vector2d u(vx, omega);

        // B_t: control input matrix (3x2), state-dependent via theta
        // Transforms u from Robot Frame to World Frame
        double theta = mu_(2);
        Eigen::Matrix<double, 3, 2> B;
        B << std::cos(theta) * dt,  0.0,
             std::sin(theta) * dt,  0.0,
             0.0,                   dt;

        // --- PREDICTION STEP ---
        // Line 2 (Thrun): mu_bar_t = A_t * mu_{t-1} + B_t * u_t
        Eigen::Vector3d mu_bar = A_ * mu_ + B * u;

        // Line 3 (Thrun): Sigma_bar_t = A_t * Sigma_{t-1} * A_t^T + R_t
        Eigen::Matrix3d Sigma_bar = A_ * Sigma_ * A_.transpose() + R_;

        // --- CORRECTION STEP ---
        // Lines 4-6 (Thrun): requires measurement z_t
        // TODO: implement when sensor is available
        // Line 4: K_t     = Sigma_bar * C^T * (C * Sigma_bar * C^T + Q)^{-1}
        // Line 5: mu_t    = mu_bar + K_t * (z_t - C * mu_bar)
        // Line 6: Sigma_t = (I - K_t * C) * Sigma_bar

        mu_    = mu_bar;
        Sigma_ = Sigma_bar;

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
        pub_->publish(msg);
    }

    Eigen::Vector3d mu_;
    Eigen::Matrix3d Sigma_, A_, R_;
    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
