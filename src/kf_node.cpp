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
 * Kalman Filter Node — Odometry + Landmark Correction
 * =====================================================
 * State:         mu = [x, y, theta]^T
 * Control input: u  = [v, omega]^T     (from /cmd_vel, robot frame)
 *
 * Prediction  (Thrun Table 3.1, Lines 2–3)
 *   mu_bar    = A * mu + B(theta) * u    — B contains cos/sin of theta
 *   Sigma_bar = A * Sigma * A^T + R
 *   NOTE: B is state-dependent — this is an approximation (linearisation).
 *         The EKF eliminates this error via the Jacobian G and nonlinear g().
 *
 * Correction 1 — Odometry  (Lines 4–6)
 *   z = [x, y, theta],  C = I_3          — always applied when available
 *
 * Correction 2 — Landmarks  (one sequential update per detected landmark)
 *   z_lm  = r_measured                   — scalar range from /scan
 *   C_lm  = dh/dmu |_{mu_}              — linearised 1×3 Jacobian of range
 *   Treated in KF as a locally-fixed linear measurement matrix.
 *   The EKF applies the same Jacobian formula — the difference is that
 *   the EKF prediction step is already exact (g, G), so the overall
 *   linearisation error is smaller.
 *
 * ref: Thrun (2006) Table 3.1 (KF) + Table 7.2 (EKF localisation, adapted)
 */

// One detected landmark for the current scan cycle
struct LandmarkObs {
    double mx, my;   // world-frame position of the landmark
    double range;    // scan range reading at the expected bearing [m]
};

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
        // Initial state
        mu_    = Eigen::Vector3d::Zero();
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;

        // System matrices (Thrun notation)
        A_ = Eigen::Matrix3d::Identity();          // state transition
        R_ = Eigen::Matrix3d::Identity() * 0.01;   // process noise
        C_ = Eigen::Matrix3d::Identity();          // odometry measurement matrix
        Q_ = Eigen::Matrix3d::Identity() * 0.1;    // odometry measurement noise

        // Landmark range noise variance  (Q_lm = sigma_r^2)
        q_lm_ = LM_SIGMA_R * LM_SIGMA_R;

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&KalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&KalmanFilterNode::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&KalmanFilterNode::scanCallback, this, std::placeholders::_1));

        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_kf", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "KF Node started (odom + landmark correction)");
    }

private:
    // ------------------------------------------------------------------
    // Cache latest odometry measurement
    // ------------------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        z_(0) = msg->pose.pose.position.x;
        z_(1) = msg->pose.pose.position.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        z_(2) = 2.0 * std::atan2(qz, qw);
        odom_received_ = true;
    }

    // ------------------------------------------------------------------
    // Detect landmarks from the latest LaserScan
    // ------------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        detected_.clear();

        for (const auto& lm : LANDMARK_MAP)
        {
            // Vector from robot to landmark in world frame
            double dx    = lm.x - mu_(0);
            double dy    = lm.y - mu_(1);
            double r_exp = std::sqrt(dx*dx + dy*dy);

            // Skip if landmark is outside sensor range
            if (r_exp < msg->range_min + 0.05 || r_exp > msg->range_max - 0.05)
                continue;

            // Expected bearing to landmark in robot frame
            double bearing = wrapAngle(std::atan2(dy, dx) - mu_(2));
            if (bearing < msg->angle_min || bearing > msg->angle_max)
                continue;

            // Index of the scan beam closest to the expected bearing
            int idx = static_cast<int>(
                std::round((bearing - msg->angle_min) / msg->angle_increment));
            if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) continue;

            float r_meas = msg->ranges[idx];
            if (!std::isfinite(r_meas)) continue;
            if (r_meas < msg->range_min || r_meas > msg->range_max) continue;

            // Gating: only accept if measured range is close to expected
            if (std::abs(r_meas - r_exp) > LM_GATE_M) continue;

            detected_.push_back({lm.x, lm.y, static_cast<double>(r_meas)});
            RCLCPP_DEBUG(this->get_logger(),
                "KF: detected %s  r_exp=%.3f  r_meas=%.3f",
                lm.name, r_exp, static_cast<double>(r_meas));
        }
    }

    // ------------------------------------------------------------------
    // Main filter step — triggered by every /cmd_vel message
    // ------------------------------------------------------------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        double v     = msg->linear.x;
        double omega = msg->angular.z;
        double theta = mu_(2);

        // ---------------------------------------------------------------
        // PREDICTION  (Thrun Table 3.1, Lines 2–3)
        // ---------------------------------------------------------------

        // B_t (3×2): maps u = [v, omega] from robot frame to world frame.
        // cos/sin make B state-dependent — a linearisation the EKF avoids.
        Eigen::Matrix<double, 3, 2> B;
        B << std::cos(theta)*dt, 0.0,
             std::sin(theta)*dt, 0.0,
             0.0,                dt;

        Eigen::Vector3d mu_bar    = A_ * mu_ + B * Eigen::Vector2d(v, omega);
        Eigen::Matrix3d Sigma_bar = A_ * Sigma_ * A_.transpose() + R_;

        // ---------------------------------------------------------------
        // CORRECTION 1: Odometry  (Lines 4–6)
        // ---------------------------------------------------------------
        if (odom_received_)
        {
            Eigen::Matrix3d S = C_ * Sigma_bar * C_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * C_.transpose() * S.inverse();

            Eigen::Vector3d innov = z_ - C_ * mu_bar;
            innov(2) = wrapAngle(innov(2));

            mu_    = mu_bar + K * innov;
            Sigma_ = (Eigen::Matrix3d::Identity() - K * C_) * Sigma_bar;
        }
        else
        {
            mu_    = mu_bar;
            Sigma_ = Sigma_bar;
        }

        // ---------------------------------------------------------------
        // CORRECTION 2: Landmarks  (sequential 1-D range updates)
        // ---------------------------------------------------------------
        //
        // Measurement function:  h(mu, m_j) = || m_j - [x, y] ||
        //
        // C_lm (1×3) is the linearised Jacobian of h w.r.t. mu,
        // evaluated at the current estimate mu_.  The KF treats this as
        // a locally-fixed linear matrix — valid while the linearisation
        // point stays close to the true pose.  If the filter drifts, the
        // approximation error grows (a limitation the EKF's exact
        // prediction step partially mitigates).
        //
        // Each landmark yields an independent scalar measurement and is
        // fused sequentially.
        for (const auto& obs : detected_)
        {
            double dx = obs.mx - mu_(0);
            double dy = obs.my - mu_(1);
            double r2 = dx*dx + dy*dy;
            double r  = std::sqrt(r2);
            if (r < 1e-4) continue;

            // Linearised measurement matrix C_lm (1×3):
            //   dh/dx     = -dx / r
            //   dh/dy     = -dy / r
            //   dh/dtheta =  0   (range does not depend on heading)
            Eigen::Matrix<double,1,3> C_lm;
            C_lm << -dx/r, -dy/r, 0.0;

            // Innovation: measured range minus expected range (scalar)
            double innov = obs.range - r;

            // Kalman gain (3×1)
            double          S_lm = (C_lm * Sigma_ * C_lm.transpose())(0,0) + q_lm_;
            Eigen::Vector3d K_lm = Sigma_ * C_lm.transpose() / S_lm;

            // State and covariance update
            mu_    += K_lm * innov;
            mu_(2)  = wrapAngle(mu_(2));
            Sigma_  = (Eigen::Matrix3d::Identity() - K_lm * C_lm) * Sigma_;
        }

        publishPose();
    }

    // ------------------------------------------------------------------
    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        auto now = this->get_clock()->now();
        msg.header.stamp.sec     = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
        msg.header.frame_id = "map";

        msg.pose.pose.position.x        = mu_(0);
        msg.pose.pose.position.y        = mu_(1);
        msg.pose.pose.orientation.z     = std::sin(mu_(2) / 2.0);
        msg.pose.pose.orientation.w     = std::cos(mu_(2) / 2.0);

        // Covariance (6×6 row-major): x→[0,0], y→[1,1], theta→[5,5]
        msg.pose.covariance[0]  = Sigma_(0,0);
        msg.pose.covariance[7]  = Sigma_(1,1);
        msg.pose.covariance[35] = Sigma_(2,2);

        pub_->publish(msg);
    }

    // State
    Eigen::Vector3d mu_;
    Eigen::Matrix3d Sigma_;
    Eigen::Vector3d z_;          // latest odometry measurement
    bool            odom_received_;

    // Matrices (Thrun notation)
    Eigen::Matrix3d A_;   // state transition
    Eigen::Matrix3d R_;   // process noise covariance
    Eigen::Matrix3d C_;   // odometry measurement matrix
    Eigen::Matrix3d Q_;   // odometry measurement noise covariance
    double          q_lm_;  // landmark range noise variance (scalar)

    // Landmark detections from the latest scan cycle
    std::vector<LandmarkObs> detected_;

    rclcpp::Time last_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}