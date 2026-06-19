#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "probabilistic_robot_lab/landmark_map.hpp"
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

/*
 * Particle Filter Node (Monte Carlo Localization)
 * ================================================
 * State:         x = [x, y, theta]^T  (per particle)
 * Control input: u = [v, omega]^T     (from /cmd_vel)
 * Measurement 1: z = [x, y, theta]   (from /odom)
 * Measurement 2: z = r_measured       (from /scan, landmark range)
 *
 * Algorithm: Thrun (2006) Table 4.3 — Particle Filter
 *
 * PREDICTION — Motion model (sampling step):
 *   x_t^m ~ p(x_t | u_t, x_{t-1}^m)
 *   Gaussian noise added to v and omega before integration.
 *
 * CORRECTION — Importance weights:
 *   w_t^m = p(z_t | x_t^m)
 *   Gaussian likelihood on odom + landmark range (same data as KF/EKF).
 *
 * RESAMPLING — Systematic resampling (Thrun Table 4.4):
 *   Controlled by ROS2 parameter 'resampling' (default: true).
 *
 * -----------------------------------------------------------------------
 * Special Task (ID 2510331009) — Particle Degeneration Analysis
 * -----------------------------------------------------------------------
 * Launch with: ros2 launch ... pf:=true resampling:=false
 *
 * Without resampling, particles cannot adapt after weight updates.
 * Low-weight particles are never replaced → weight collapse.
 *
 * Effective Sample Size (N_eff):
 *   N_eff = 1 / sum(w_i^2)
 *   Range: [1, N]
 *   N_eff → N : uniform weights, healthy diversity
 *   N_eff → 1 : all weight on one particle (full degeneration)
 *
 * Published on /n_eff for logging and comparison plots.
 * -----------------------------------------------------------------------
 *
 * Tunable via ROS2 parameters (set in launch file for experiments):
 *   num_particles     (default 500)
 *   resampling        (default true)
 *   sigma_process_v   (default 0.05)  — motion noise: linear velocity
 *   sigma_process_w   (default 0.05)  — motion noise: angular velocity
 *   sigma_meas_xy     (default 0.10)  — odom position noise
 *   sigma_meas_theta  (default 0.10)  — odom heading noise
 *
 * ref: Thrun, Burgard, Fox — Probabilistic Robotics (2006), Ch. 4
 */

static inline double wrapAngle(double a)
{
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

static inline double gaussianLikelihood(double err, double sigma)
{
    return std::exp(-0.5 * (err*err) / (sigma*sigma));
    // Omit normalisation constant — only ratios matter for resampling
}

struct Particle { double x, y, theta, weight; };

struct LandmarkObs { double mx, my, range; };

// ============================================================

class ParticleFilterNode : public rclcpp::Node
{
public:
    ParticleFilterNode()
    : Node("pf_node"),
      odom_received_(false),
      rng_(std::random_device{}())
    {
        // ---- Parameters ----
        this->declare_parameter("num_particles",    500);
        this->declare_parameter("resampling",       true);
        this->declare_parameter("sigma_process_v",  0.05);
        this->declare_parameter("sigma_process_w",  0.05);
        this->declare_parameter("sigma_meas_xy",    0.10);
        this->declare_parameter("sigma_meas_theta", 0.10);

        N_              = this->get_parameter("num_particles").as_int();
        do_resampling_  = this->get_parameter("resampling").as_bool();
        sigma_v_        = this->get_parameter("sigma_process_v").as_double();
        sigma_omega_    = this->get_parameter("sigma_process_w").as_double();
        sigma_odom_xy_  = this->get_parameter("sigma_meas_xy").as_double();
        sigma_odom_th_  = this->get_parameter("sigma_meas_theta").as_double();

        // ---- Initialise particles at origin ----
        particles_.resize(N_);
        for (auto& p : particles_) {
            p.x = p.y = p.theta = 0.0;
            p.weight = 1.0 / N_;
        }

        // ---- Subscribers ----
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ParticleFilterNode::cmdVelCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));

        // ---- Publishers ----
        pose_pub_      = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose_pf", 10);
        particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/pf_particles", 10);
        n_eff_pub_     = this->create_publisher<std_msgs::msg::Float64>(
            "/n_eff", 10);

        last_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(),
            "PF Node started  N=%d  resampling=%s  "
            "sigma_v=%.3f  sigma_w=%.3f  sigma_odom_xy=%.3f  sigma_odom_th=%.3f",
            N_, do_resampling_ ? "true" : "false (Special Task)",
            sigma_v_, sigma_omega_, sigma_odom_xy_, sigma_odom_th_);
    }

private:
    // ----------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        z_odom_(0) = msg->pose.pose.position.x;
        z_odom_(1) = msg->pose.pose.position.y;
        z_odom_(2) = 2.0 * std::atan2(
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        last_odom_time_ = this->get_clock()->now();
        odom_received_  = true;
    }

    // ----------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Use current weighted-mean pose to find expected bearing
        auto [mx, my, mth] = weightedMean();
        detected_lm_.clear();

        for (const auto& lm : LANDMARK_MAP) {
            double dx    = lm.x - mx;
            double dy    = lm.y - my;
            double r_exp = std::sqrt(dx*dx + dy*dy);

            if (r_exp < msg->range_min + 0.05 || r_exp > msg->range_max - 0.05) continue;

            double bearing = wrapAngle(std::atan2(dy, dx) - mth);
            if (bearing < msg->angle_min || bearing > msg->angle_max) continue;

            int idx = static_cast<int>(
                std::round((bearing - msg->angle_min) / msg->angle_increment));
            if (idx < 0 || idx >= static_cast<int>(msg->ranges.size())) continue;

            float r_meas = msg->ranges[idx];
            if (!std::isfinite(r_meas)) continue;
            if (r_meas < msg->range_min || r_meas > msg->range_max) continue;
            if (std::abs(r_meas - r_exp) > LM_GATE_M) continue;

            detected_lm_.push_back({lm.x, lm.y, static_cast<double>(r_meas)});
        }
    }

    // ----------------------------------------------------------
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        double v     = msg->linear.x;
        double omega = msg->angular.z;

        std::normal_distribution<double> dv(0.0, sigma_v_);
        std::normal_distribution<double> dw(0.0, sigma_omega_);

        // -------------------------------------------------------
        // PREDICTION — sample x_t^m ~ p(x_t | u_t, x_{t-1}^m)
        // -------------------------------------------------------
        for (auto& p : particles_) {
            double vn = v     + dv(rng_);
            double wn = omega + dw(rng_);
            p.x     += vn * std::cos(p.theta) * dt;
            p.y     += vn * std::sin(p.theta) * dt;
            p.theta  = wrapAngle(p.theta + wn * dt);
        }

        // -------------------------------------------------------
        // CORRECTION — importance weights w_t^m = p(z_t | x_t^m)
        // -------------------------------------------------------
        bool odom_valid = odom_received_ &&
            (this->get_clock()->now() - last_odom_time_).seconds() < 0.5;

        if (odom_valid || !detected_lm_.empty()) {
            for (auto& p : particles_) {
                double w = 1.0;

                // Odometry measurement likelihood
                if (odom_valid) {
                    w *= gaussianLikelihood(z_odom_(0) - p.x,     sigma_odom_xy_);
                    w *= gaussianLikelihood(z_odom_(1) - p.y,     sigma_odom_xy_);
                    w *= gaussianLikelihood(wrapAngle(z_odom_(2) - p.theta), sigma_odom_th_);
                }

                // Landmark range likelihood
                for (const auto& obs : detected_lm_) {
                    double dx    = obs.mx - p.x;
                    double dy    = obs.my - p.y;
                    double r_exp = std::sqrt(dx*dx + dy*dy);
                    w *= gaussianLikelihood(obs.range - r_exp, LM_SIGMA_R);
                }

                p.weight *= std::max(w, 1e-300);   // guard against underflow
            }

            normalizeWeights();
        }

        // -------------------------------------------------------
        // RESAMPLING — systematic resampling (Thrun Table 4.4)
        // Disabled for Special Task to demonstrate degeneration.
        // -------------------------------------------------------
        if (do_resampling_) {
            systematicResample();
        }

        // Effective sample size
        //   N_eff = 1 / sum(w_i^2)   ∈ [1, N]
        //   Tracks particle diversity; published for Special Task analysis
        double sum_w2 = 0.0;
        for (const auto& p : particles_) sum_w2 += p.weight * p.weight;
        double n_eff = (sum_w2 > 1e-300) ? 1.0 / sum_w2 : 0.0;

        auto neff_msg = std_msgs::msg::Float64();
        neff_msg.data = n_eff;
        n_eff_pub_->publish(neff_msg);

        publishPose();
        publishParticles();
    }

    // ----------------------------------------------------------
    // Systematic resampling — O(N), low variance
    void systematicResample()
    {
        // Build cumulative sum
        std::vector<double> cum(N_);
        cum[0] = particles_[0].weight;
        for (int i = 1; i < N_; ++i)
            cum[i] = cum[i-1] + particles_[i].weight;

        std::uniform_real_distribution<double> ud(0.0, 1.0 / N_);
        double r = ud(rng_);

        std::vector<Particle> resampled;
        resampled.reserve(N_);
        int j = 0;
        for (int i = 0; i < N_; ++i) {
            double u = r + static_cast<double>(i) / N_;
            while (j < N_-1 && cum[j] < u) ++j;
            resampled.push_back(particles_[j]);
            resampled.back().weight = 1.0 / N_;
        }
        particles_ = std::move(resampled);
    }

    // ----------------------------------------------------------
    void normalizeWeights()
    {
        double total = 0.0;
        for (const auto& p : particles_) total += p.weight;
        if (total < 1e-300) {
            RCLCPP_WARN(this->get_logger(),
                "PF: complete weight collapse — resetting to uniform");
            for (auto& p : particles_) p.weight = 1.0 / N_;
            return;
        }
        for (auto& p : particles_) p.weight /= total;
    }

    // ----------------------------------------------------------
    std::tuple<double,double,double> weightedMean() const
    {
        double mx = 0, my = 0, sin_s = 0, cos_s = 0;
        for (const auto& p : particles_) {
            mx    += p.weight * p.x;
            my    += p.weight * p.y;
            sin_s += p.weight * std::sin(p.theta);
            cos_s += p.weight * std::cos(p.theta);
        }
        return {mx, my, std::atan2(sin_s, cos_s)};
    }

    // ----------------------------------------------------------
    void publishPose()
    {
        auto [mx, my, mth] = weightedMean();

        // Weighted variance (= covariance diagonal)
        double vx = 0, vy = 0, vt = 0;
        for (const auto& p : particles_) {
            vx += p.weight * (p.x - mx) * (p.x - mx);
            vy += p.weight * (p.y - my) * (p.y - my);
            double dt = wrapAngle(p.theta - mth);
            vt += p.weight * dt * dt;
        }

        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        auto now = this->get_clock()->now();
        msg.header.stamp.sec     = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
        msg.header.frame_id = "odom";

        msg.pose.pose.position.x    = mx;
        msg.pose.pose.position.y    = my;
        msg.pose.pose.orientation.z = std::sin(mth / 2.0);
        msg.pose.pose.orientation.w = std::cos(mth / 2.0);

        msg.pose.covariance[0]  = vx;
        msg.pose.covariance[7]  = vy;
        msg.pose.covariance[35] = vt;

        pose_pub_->publish(msg);
    }

    // ----------------------------------------------------------
    void publishParticles()
    {
        auto msg = geometry_msgs::msg::PoseArray();
        auto now = this->get_clock()->now();
        msg.header.stamp.sec     = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
        msg.header.frame_id = "odom";
        msg.poses.reserve(N_);

        for (const auto& p : particles_) {
            geometry_msgs::msg::Pose pose;
            pose.position.x    = p.x;
            pose.position.y    = p.y;
            pose.orientation.z = std::sin(p.theta / 2.0);
            pose.orientation.w = std::cos(p.theta / 2.0);
            msg.poses.push_back(pose);
        }
        particles_pub_->publish(msg);
    }

    // ---- State ----
    std::vector<Particle> particles_;
    int    N_;
    bool   do_resampling_;
    bool   odom_received_;

    Eigen::Vector3d  z_odom_;
    rclcpp::Time     last_odom_time_;
    std::vector<LandmarkObs> detected_lm_;

    // ---- Noise parameters ----
    double sigma_v_, sigma_omega_;
    double sigma_odom_xy_, sigma_odom_th_;

    std::mt19937   rng_;
    rclcpp::Time   last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    particles_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           n_eff_pub_;
};

// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
