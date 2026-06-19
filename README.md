# Probabilistic Robot Lab (PROLB_R)

ROS2 implementation of **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** for mobile robot localization on a TurtleBot3 simulation.

**Course:** Probabilistic Robotics (PRO), FH Technikum Wien  
**Special Task (2510331009):** Disable resampling in the PF and analyze particle degeneration.  
**Algorithm reference:** Thrun, Burgard & Fox — *Probabilistic Robotics* (2006)

---

## Environment

| | |
|---|---|
| OS | Ubuntu 24.04 (WSL2 / native) |
| ROS2 | Jazzy |
| Simulation | TurtleBot3 Waffle + Nav2 (Gazebo) |
| Language | C++ (rclcpp, Eigen3) + Python 3 |

---

## Prerequisites

```bash
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2-bringup libeigen3-dev python3-pip
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc && source ~/.bashrc
pip install matplotlib pandas --break-system-packages
```

---

## Setup

```bash
mkdir -p ~/prob_ros_ws/src && cd ~/prob_ros_ws/src
git clone git@github.com:Raini20/PROLB_R.git probabilistic_robot_lab
cd ~/prob_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Add to `~/.bashrc` so you don't need to source on every terminal:
```bash
echo "source ~/prob_ros_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running

### Default (KF + EKF, auto navigation)

```bash
ros2 launch probabilistic_robot_lab filters.launch.py
```

Everything starts automatically in the correct order:
- **0 s** — Gazebo + Nav2 + AMCL
- **5 s** — Filter nodes + RViz + Data Logger
- **10 s** — AutoNav: sets initial pose and drives the robot through fixed waypoints

RViz opens automatically with the configured displays for `/pose_kf`, `/pose_ekf`.

### With Particle Filter

```bash
ros2 launch probabilistic_robot_lab filters.launch.py pf:=true
```

### Special Task — Particle Degeneration (no resampling)

```bash
ros2 launch probabilistic_robot_lab filters.launch.py pf:=true resampling:=false
```

### Noise Experiments (mandatory — vary Q and R)

```bash
# High process noise (low trust in motion model)
ros2 launch probabilistic_robot_lab filters.launch.py sigma_process:=0.1

# High measurement noise (low trust in odometry)
ros2 launch probabilistic_robot_lab filters.launch.py sigma_meas:=0.5

# Both varied
ros2 launch probabilistic_robot_lab filters.launch.py sigma_process:=0.1 sigma_meas:=0.5
```

### Plotting results

After stopping the launch (`Ctrl+C`), the logger has saved a CSV automatically:

```bash
# All filter comparison plots (Trajectory, Error, RMSE, Covariance)
python3 ~/prob_ros_ws/src/probabilistic_robot_lab/scripts/plot_results.py

# Special Task: N_eff comparison with/without resampling
python3 ~/prob_ros_ws/src/probabilistic_robot_lab/scripts/plot_n_eff.py \
    ~/prob_ros_ws/logs/filter_data_WITH_resampling.csv \
    ~/prob_ros_ws/logs/filter_data_NO_resampling.csv
```

Logs are saved to `~/prob_ros_ws/logs/`.

---

## Nodes

| Node | Executable | Subscribes | Publishes |
|---|---|---|---|
| Kalman Filter | `kf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_kf` |
| Extended KF | `ekf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_ekf` |
| Particle Filter | `pf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_pf`, `/pf_particles`, `/n_eff` |
| Data Logger | `data_logger` | `/odom`, `/pose_kf`, `/pose_ekf`, `/pose_pf`, `/n_eff` | CSV file |
| AutoNav | `auto_nav` | — | `/initialpose`, Nav2 goals |

**State vector (all filters):** `mu = [x, y, θ]ᵀ`  
**Reference frame:** `odom`

---

## Filter Overview

### Kalman Filter (KF)

Prediction uses a **linearised** B matrix (state-dependent cos/sin — approximation).  
Correction uses identity measurement matrix `C = I₃` for odometry, plus a linearised Jacobian `C_lm` for landmark range.

### Extended Kalman Filter (EKF)

Prediction uses the **exact nonlinear** motion function `g(u, μ)` and Jacobian `G = ∂g/∂μ`.  
Correction: odometry via `H = I₃`, landmarks via Jacobian `H_j = ∂h/∂μ`.  
Key advantage over KF: no linearisation error in the prediction step.

### Particle Filter (PF)

Monte Carlo Localization — Thrun Table 4.3.  
Motion model: sample `x_t^m ~ p(x_t | u_t, x_{t-1}^m)` with Gaussian noise on `v` and `ω`.  
Measurement weights: Gaussian likelihood on odom + landmark range.  
Resampling: systematic resampling (Thrun Table 4.4).

---

## Landmarks

Landmarks are shared across all three filters for fair comparison.  
Defined in `include/probabilistic_robot_lab/landmark_map.hpp`.

To set landmark positions:
1. Run the simulation and open RViz.
2. Use the **Publish Point** tool (toolbar) — click on visible obstacles.
3. Read coordinates from `/clicked_point`: `ros2 topic echo /clicked_point --once`
4. Update `LANDMARK_MAP` in `landmark_map.hpp` and rebuild.

---

## Special Task (ID 2510331009) — Particle Degeneration

Without resampling, low-weight particles are never replaced.  
Over time, all probability mass concentrates on a single particle.

**Effective Sample Size:**  
`N_eff = 1 / Σ wᵢ²`  — ranges from `1` (full degeneration) to `N` (uniform weights)

**Experiment:**
1. Run with resampling (baseline): `pf:=true resampling:=true` → save log A
2. Run without resampling: `pf:=true resampling:=false` → save log B
3. Compare: `python3 plot_n_eff.py log_A.csv log_B.csv`

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `sigma_process` | `0.01` | Process noise std dev → scales R |
| `sigma_meas` | `0.10` | Measurement noise std dev → scales Q |
| `num_particles` | `500` | Number of PF particles |
| `resampling` | `true` | Enable resampling (`false` = Special Task) |

---

## Status

### Implementation

| | KF | EKF | PF |
|---|---|---|---|
| Prediction step | ✅ | ✅ | ✅ |
| Correction step (odom) | ✅ | ✅ | ✅ |
| Correction step (landmarks) | ✅ | ✅ | ✅ |
| Publish `/pose_*` | ✅ | ✅ | ✅ |
| RViz visualization | ✅ | ✅ | ✅ |
| Timestamp check (stale odom) | ✅ | ✅ | ✅ |
| Tunable Q/R parameters | ✅ | ✅ | ✅ |

### Experiments

| Experiment | Status |
|---|---|
| Process noise (σ_process) variation | 🔲 |
| Measurement noise (σ_meas) variation | 🔲 |
| Ground truth RMSE vs `/odom` | ✅ (plot_results.py) |
| Runtime comparison KF / EKF / PF | 🔲 |
| Landmark detection (shared map) | ✅ |
| Covariance trace visualization | ✅ |

### Special Task (2510331009)

| Task | Status |
|---|---|
| PF baseline with standard resampling | ✅ |
| Disabled resampling → degeneration | ✅ |
| N_eff logging | ✅ |
| N_eff comparison plot | ✅ (plot_n_eff.py) |
| RMSE comparison with/without resampling | ✅ (plot_n_eff.py) |

### Submission

| | Status |
|---|---|
| GitHub repo + README | ✅ |
| Paper (documentation) | 🔲 |
| PowerPoint presentation | 🔲 |

---

## References

- Thrun, S., Burgard, W., Fox, D. (2006). *Probabilistic Robotics*. MIT Press.
  - Table 3.1 — Kalman Filter
  - Table 3.3 — Extended Kalman Filter
  - Table 4.3 — Particle Filter (MCL)
  - Table 4.4 — Systematic Resampling
  - Table 7.2 — EKF Localization with Landmarks
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy)
