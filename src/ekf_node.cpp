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
 *   Sigma_bar = G * Sigma * G^T + R   — G = dg/dmu (Jacobian of g)
 *   This eliminates the linearisation error present in the KF's B matrix.
 *
 * Correction 1 — Odometry  (Lines 4–6, H = I_3)
 *   z = [x, y, theta]                  — always applied when available
 *
 * Correction 2 — Landmarks  (Thrun Table 7.2, adapted)
 *   h(mu, m_j) = || m_j - [x, y] ||   — nonlinear range measurement
 *   H_j = dh/dmu |_{mu_}              — 1×3 Jacobian of h  (= C_lm in KF)
 *   One sequential EKF update per detected landmark.
 *
 * Key difference to KF:
 *   • Prediction: g() and G replace the approximated A, B matrices.
 *     The motion model is exact; only the covariance propagation is
 *     linearised (first-order Taylor — unavoidable in EKF).
 *   • Landmark correction: H_j is evaluated at the post-correction mu_,
 *     which is the current best estimate.  Both KF and EKF use the same
 *     Jacobian formula; the EKF's lower prediction error makes the
 *     linearisation point more accurate.
 *
 * ref: Thrun (2006) Table 3.3 (EKF) + Table 7.2 (EKF localisation)
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

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("ekf_node"), odom_received_(false)
    {
        // Initial state
        mu_    = Eigen::Vector3d::Zero();
        Sigma_ = Eigen::Matrix3d::Identity() * 0.1;

        // R_t: process noise covariance  (Line 3)
        R_ = Eigen::Matrix3d::Identity() * 0.01;

        // H_t for odometry correction: h(mu) = mu  →  H = I_3
        H_ = Eigen::Matrix3d::Identity();

        // Q_t: odometry measurement noise covariance  (Line 4)
        Q_ = Eigen::Matrix3d::Identity() * 0.1;

        // Landmark range noise variance  (Q_lm = sigma_r^2)
        q_lm_ = LM_SIGMA_R * LM_SIGMA_R;

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ExtendedKalmanFilterNode::cmdVelCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ExtendedKalmanFilterNode::odomCallback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ExtendedKalmanFilterNode::scanCallback, this, std::placeholders::_1));

        // Publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_ekf", 10);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "EKF Node started (odom + landmark correction)");
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
    // (Identical detection logic to KF — same input data for both.)
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
                "EKF: detected %s  r_exp=%.3f  r_meas=%.3f",
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
        // PREDICTION  (Thrun Table 3.3, Lines 1–3)
        // ---------------------------------------------------------------

        // Line 1: mu_bar = g(u, mu)  — exact nonlinear motion model
        Eigen::Vector3d mu_bar;
        mu_bar(0) = mu_(0) + v * std::cos(theta) * dt;
        mu_bar(1) = mu_(1) + v * std::sin(theta) * dt;
        mu_bar(2) = mu_(2) + omega * dt;

        // Line 2: G = dg/dmu  — Jacobian of g w.r.t. mu, evaluated at mu_{t-1}
        //   G = [[1,  0,  -v*sin(theta)*dt],
        //        [0,  1,   v*cos(theta)*dt],
        //        [0,  0,   1             ]]
        Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
        G(0,2) = -v * std::sin(theta) * dt;
        G(1,2) =  v * std::cos(theta) * dt;

        // Line 3: Sigma_bar = G * Sigma * G^T + R
        Eigen::Matrix3d Sigma_bar = G * Sigma_ * G.transpose() + R_;

        // ---------------------------------------------------------------
        // CORRECTION 1: Odometry  (Lines 4–6)
        // h(mu) = mu  →  H = I_3  (identity, linear)
        // ---------------------------------------------------------------
        if (odom_received_)
        {
            Eigen::Matrix3d S = H_ * Sigma_bar * H_.transpose() + Q_;
            Eigen::Matrix3d K = Sigma_bar * H_.transpose() * S.inverse();

            Eigen::Vector3d innov = z_ - mu_bar;
            innov(2) = wrapAngle(innov(2));

            mu_    = mu_bar + K * innov;
            Sigma_ = (Eigen::Matrix3d::Identity() - K * H_) * Sigma_bar;
        }
        else
        {
            mu_    = mu_bar;
            Sigma_ = Sigma_bar;
        }

        // ---------------------------------------------------------------
        // CORRECTION 2: Landmarks  (Thrun Table 7.2, adapted)
        // ---------------------------------------------------------------
        //
        // Nonlinear measurement function:
        //   h(mu, m_j) = || m_j - [x, y] ||   (Euclidean range)
        //
        // Jacobian H_j (1×3):
        //   dh/dmu = [-dx/r,  -dy/r,  0]
        //
        // Each landmark yields an independent scalar observation.
        // Sequential updates are applied directly to mu_ and Sigma_
        // (equivalent to a joint update for independent observations).
        //
        // H_j uses the same mathematical form as C_lm in the KF.
        // The justification is different: in the EKF this IS the correct
        // first-order linearisation of h.  In the KF it is an ad-hoc
        // approximation layered on top of an already-approximate
        // prediction step.
        for (const auto& obs : detected_)
        {
            double dx = obs.mx - mu_(0);
            double dy = obs.my - mu_(1);
            double r2 = dx*dx + dy*dy;
            double r  = std::sqrt(r2);
            if (r < 1e-4) continue;

            // Jacobian H_j (1×3) of h = ||m - [x,y]||  w.r.t.  mu
            Eigen::Matrix<double,1,3> H_j;
            H_j << -dx/r, -dy/r, 0.0;

            // Innovation: measured range minus expected range (scalar)
            double innov = obs.range - r;

            // Kalman gain (3×1)
            double          S_j = (H_j * Sigma_ * H_j.transpose())(0,0) + q_lm_;
            Eigen::Vector3d K_j = Sigma_ * H_j.transpose() / S_j;

            // State and covariance update
            mu_    += K_j * innov;
            mu_(2)  = wrapAngle(mu_(2));
            Sigma_  = (Eigen::Matrix3d::Identity() - K_j * H_j) * Sigma_;
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

    // Noise matrices (Thrun notation)
    Eigen::Matrix3d R_;    // process noise covariance
    Eigen::Matrix3d H_;    // odometry measurement Jacobian (I_3)
    Eigen::Matrix3d Q_;    // odometry measurement noise covariance
    double          q_lm_; // landmark range noise variance (scalar)

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
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}