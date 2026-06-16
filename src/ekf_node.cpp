#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <cmath>
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*
 * Extended Kalman Filter Node
 * State:         mu = [x, y, theta]^T
 * Control input: u  = [v_x, omega]^T  (from /cmd_vel, Robot Frame)
 * Measurement:   z  = [x, y, theta]^T (from /odom, World Frame)
 *
 * Algorithm EKF(mu_{t-1}, Sigma_{t-1}, u_t, z_t)
 * ref: Thrun (2006), Probabilistic Robotics, Table 3.3
 *
 * Key difference to KF:
 *   KF uses linear matrices A and B.
 *   EKF uses the nonlinear motion function g(mu, u) directly for the state
 *   update, and its Jacobian G_t for the covariance propagation.
 *   This correctly handles the nonlinearity introduced by cos(theta)/sin(theta).
 *
 * Thrun notation:
 *   g(u_t, mu_{t-1}) = nonlinear motion model
 *   G_t              = Jacobian of g w.r.t. mu (linearization around current state)
 *   R_t              = process noise covariance
 *   H_t              = Jacobian of measurement model h (= Identity here)
 *   Q_t              = measurement noise covariance
 */

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("ekf_node"), odom_received_(false)
    {
        // --- Initial State ---
        mu_    = Eigen::Vector3d::Zero();
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;

        // R_t: process noise covariance (Thrun notation, Line 3)
        R_ = Eigen::Matrix3d::Identity() * 0.01;

        // H_t: Jacobian of measurement model h(mu) = mu  =>  H = Identity
        // Line 4: K_t = Sigma_bar * H^T * (H * Sigma_bar * H^T + Q_t)^{-1}
        H_ = Eigen::Matrix3d::Identity();

        // Q_t: measurement noise covariance (Thrun notation, Line 4)
        Q_ = Eigen::Matrix3d::Identity() * 0.1;

        // --- Subscribers ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ExtendedKalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ExtendedKalmanFilterNode::odomCallback, this, std::placeholders::_1));

        // --- Publisher ---
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_ekf", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "EKF Node started");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        z_(0) = msg->pose.pose.position.x;
        z_(1) = msg->pose.pose.position.y;
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

        if (dt <= 0.0 || dt > 1.0) return;

        double v     = msg->linear.x;
        double omega = msg->angular.z;
        double theta = mu_(2);

        // -------------------------------------------------------
        // PREDICTION STEP
        // -------------------------------------------------------

        // Line 1 (Thrun): mu_bar = g(u_t, mu_{t-1})
        // Nonlinear motion model — no linearization needed here
        Eigen::Vector3d mu_bar;
        mu_bar(0) = mu_(0) + v * std::cos(theta) * dt;
        mu_bar(1) = mu_(1) + v * std::sin(theta) * dt;
        mu_bar(2) = mu_(2) + omega * dt;

        // Line 2 (Thrun): G_t = Jacobian of g w.r.t. mu, evaluated at mu_{t-1}
        // dg/dmu = [[1, 0, -v*sin(theta)*dt],
        //           [0, 1,  v*cos(theta)*dt ],
        //           [0, 0,  1              ]]
        Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
        G(0, 2) = -v * std::sin(theta) * dt;
        G(1, 2) =  v * std::cos(theta) * dt;

        // Line 3 (Thrun): Sigma_bar = G_t * Sigma_{t-1} * G_t^T + R_t
        Eigen::Matrix3d Sigma_bar = G * Sigma_ * G.transpose() + R_;

        // -------------------------------------------------------
        // CORRECTION STEP
        // -------------------------------------------------------
        if (odom_received_)
        {
            // Line 4 (Thrun): K_t = Sigma_bar * H^T * (H * Sigma_bar * H^T + Q_t)^{-1}
            Eigen::Matrix3d S = H_ * Sigma_bar * H_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * H_.transpose() * S.inverse();

            // Line 5 (Thrun): mu_t = mu_bar + K_t * (z_t - h(mu_bar))
            // h(mu) = mu  (identity measurement model)
            Eigen::Vector3d innovation = z_ - mu_bar;

            // Wrap theta innovation to [-pi, pi]
            while (innovation(2) >  M_PI) innovation(2) -= 2.0 * M_PI;
            while (innovation(2) < -M_PI) innovation(2) += 2.0 * M_PI;

            mu_ = mu_bar + K * innovation;

            // Line 6 (Thrun): Sigma_t = (I - K_t * H_t) * Sigma_bar
            Sigma_ = (Eigen::Matrix3d::Identity() - K * H_) * Sigma_bar;
        }
        else
        {
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

        msg.pose.covariance[0]  = Sigma_(0, 0);
        msg.pose.covariance[7]  = Sigma_(1, 1);
        msg.pose.covariance[35] = Sigma_(2, 2);

        pub_->publish(msg);
    }

    Eigen::Vector3d mu_;
    Eigen::Matrix3d Sigma_;
    Eigen::Vector3d z_;
    bool odom_received_;

    Eigen::Matrix3d R_, H_, Q_;

    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
