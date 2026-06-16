# Probabilistic Robot Lab (PROLB_R)

ROS2 implementation of **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** for mobile robot localization on a TurtleBot3 simulation.

**Course:** Probabilistic Robotics (PRO), FH Technikum Wien  
**Special Task (2510331009):** Disable/modify resampling in the PF and analyze particle degeneration.

---

## Environment

| | |
|---|---|
| OS | Ubuntu 24.04 (WSL2) |
| ROS2 | Jazzy |
| Simulation | TurtleBot3 Waffle + Nav2 (Gazebo) |
| Language | C++ (rclcpp, Eigen3) |

---

## Prerequisites

Install TurtleBot3 and Nav2 if not already done:

```bash
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2-bringup libeigen3-dev
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc && source ~/.bashrc
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

> **Tip:** Add `source ~/prob_ros_ws/install/setup.bash` to `~/.bashrc` so you don't need to source manually each time.

---

## Running

### Terminal 1 — Simulation

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

**In RViz** (do this in order):
1. Click **2D Pose Estimate** and place the robot on the map
2. Wait ~30 s for Nav2 to become active (particle cloud appears)
3. Click **Nav2 Goal** to drive the robot

### Terminal 2 — Filter nodes

```bash
ros2 launch probabilistic_robot_lab filters.launch.py
```

This starts the **KF node** by default. Once EKF and PF are implemented:

```bash
ros2 launch probabilistic_robot_lab filters.launch.py ekf:=true pf:=true
```

---

## Nodes

| Node | Executable | Subscribes | Publishes |
|---|---|---|---|
| Kalman Filter | `kf_node` | `/cmd_vel`, `/odom` | `/pose_kf` |
| Extended KF | `ekf_node` | `/cmd_vel`, `/odom` | `/pose_ekf` |
| Particle Filter | `pf_node` | `/cmd_vel`, `/odom` | `/pose_pf` |

State vector for all filters: `[x, y, θ]`

Notation follows Thrun (2006):
- `R` = process noise covariance
- `Q` = measurement noise covariance

---

## Status

### Implementation

| | KF | EKF | PF |
|---|---|---|---|
| Prediction step | ✅ | 🔲 | 🔲 |
| Correction step | ✅ | 🔲 | 🔲 |
| Publish `/pose_*` | ✅ | 🔲 | 🔲 |
| RViz visualization | ✅ | 🔲 | 🔲 |

### Experiments

| Experiment | Status |
|---|---|
| Process noise (R) variation | 🔲 |
| Measurement noise (Q) variation | 🔲 |
| Ground truth evaluation (RMSE vs `/odom`) | 🔲 |
| Runtime comparison KF / EKF / PF | 🔲 |
| Landmark detection | 🔲 |

### Special Task (2510331009)

| Task | Status |
|---|---|
| PF baseline with standard resampling | 🔲 |
| Disabled resampling — particle degeneration | 🔲 |
| Reduced resampling frequency — N_eff over time | 🔲 |
| Adaptive resampling (N_eff < threshold) | 🔲 |
| Weight distribution / N_eff plots | 🔲 |
| RMSE comparison: full / reduced / no resampling | 🔲 |

### Submission

| | Status |
|---|---|
| GitHub repo + README | ✅ |
| Paper (documentation) | 🔲 |
| PowerPoint presentation | 🔲 |

---

## References

- Thrun, S., Burgard, W., Fox, D. (2006). *Probabilistic Robotics*. MIT Press.
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy)
