# Probabilistic Robot Lab (PROLB_R)

ROS2 implementation of **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** for mobile robot localization, applied to a TurtleBot3 simulation with Nav2.

**Course:** Probabilistic Robotics (PRO), FH Technikum Wien  
**Special Task (2510331009):** Disable or modify resampling in the PF and analyze particle degeneration.

---

## Environment

| | |
|---|---|
| OS | Ubuntu 24.04 (WSL2) |
| ROS2 | Jazzy |
| Simulation | TurtleBot3 + Nav2 (Gazebo Modern) |
| Language | C++ (rclcpp, Eigen3) |

---

## Setup

```bash
# 1. Clone into ROS2 workspace
mkdir -p ~/prob_ros_ws/src && cd ~/prob_ros_ws/src
git clone git@github.com:Raini20/PROLB_R.git probabilistic_robot_lab

# 2. Build
cd ~/prob_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

```bash
# Terminal 1 — Start TB3 simulation with Nav2
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

In RViz:
1. Set **2D Pose Estimate** to initialize AMCL
2. Wait for Nav2 to become active (~30s)
3. Set a **Nav2 Goal** to drive the robot

```bash
# Terminal 2 — Run KF node
source /opt/ros/jazzy/setup.bash
source ~/prob_ros_ws/install/setup.bash
ros2 run probabilistic_robot_lab kf_node
```

---

## Nodes

### `kf_node` — Kalman Filter

Implements the linear Kalman Filter (Thrun 2006, Table 3.1).

**Subscriptions:**

| Topic | Type | Role |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Control input u_t |
| `/odom` | `nav_msgs/Odometry` | Measurement z_t |

**Publications:**

| Topic | Type |
|---|---|
| `/pose_kf` | `geometry_msgs/PoseWithCovarianceStamped` |

**State vector:** `mu = [x, y, theta]`

**Prediction (Lines 2–3):**

```
mu_bar    = A * mu + B(theta) * u
Sigma_bar = A * Sigma * A^T + R
```

The control input `u = [v_x, omega]` arrives in the **Robot Frame** (forward/turn). To predict the next position in the **World Frame**, the velocity must be rotated by the current heading angle theta:

```
dx = v_x * cos(theta) * dt
dy = v_x * sin(theta) * dt
```

This means B contains `cos(theta)` and `sin(theta)`, which depend on the current state. Strictly speaking, this makes the motion model nonlinear — the standard KF assumes B is constant. As a pragmatic approximation, B is recomputed at each step using the latest theta estimate. The lecturer confirmed this approach is acceptable for the KF. The EKF addresses this properly by computing the Jacobian of the motion model.

**Correction (Lines 4–6):**

```
K     = Sigma_bar * C^T * (C * Sigma_bar * C^T + Q)^{-1}
mu    = mu_bar + K * (z - C * mu_bar)
Sigma = (I - K * C) * Sigma_bar
```

Measurement matrix `C = Identity` since `z = [x, y, theta]` maps directly to the state.

**Notation (Thrun):**
- `R` = process noise covariance
- `Q` = measurement noise covariance

---

## Status

| Filter | Prediction | Correction | Visualized |
|---|---|---|---|
| KF  | ✅ | ✅ | ✅ |
| EKF | 🔲 | 🔲 | 🔲 |
| PF  | 🔲 | 🔲 | 🔲 |

---

## References

- Thrun, S., Burgard, W., Fox, D. (2006). *Probabilistic Robotics*. MIT Press.
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy)
