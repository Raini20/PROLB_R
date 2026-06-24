# Probabilistic Robot Lab (PROLB_R)

ROS 2 implementation of **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** for mobile robot localization on a TurtleBot3 simulation.

**Course:** Probabilistic Robotics (PRO), FH Technikum Wien  
**Special Task (2510331009):** Disable resampling in the PF and analyze particle degeneration.  
**Algorithm reference:** Thrun, Burgard & Fox — *Probabilistic Robotics* (2006)

All experiment commands, raw result locations, and the full Q/R study summary table
are documented in **[`EXPERIMENT_LOG.md`](EXPERIMENT_LOG.md)**.

---

## Environment

| | |
|---|---|
| OS | Ubuntu 24.04 (WSL2 / native) |
| ROS 2 | Jazzy |
| Simulation | TurtleBot3 Waffle + Nav2 (Gazebo) |
| Language | C++ (rclcpp, Eigen3) + Python 3 |

---

## Prerequisites

```bash
sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2-bringup libeigen3-dev python3-pip
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc && source ~/.bashrc
python3 -m pip install matplotlib pandas --break-system-packages
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

Add to `~/.bashrc` so sourcing is not required on every new terminal:

```bash
echo "source ~/prob_ros_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running

### Full automated experiment pipeline (recommended)

Runs both experiments (resampling ON → OFF) back-to-back, saves CSVs, and generates all paper plots automatically:

```bash
cd ~/prob_ros_ws/src/probabilistic_robot_lab
./scripts/run_experiments.sh
```

**Options:**

```
./scripts/run_experiments.sh                    # both runs, then plot
./scripts/run_experiments.sh --with-resampling  # resampling-ON run only
./scripts/run_experiments.sh --without-resampling  # resampling-OFF run only
./scripts/run_experiments.sh --timeout=600      # extend nav timeout to 600 s
./scripts/run_experiments.sh --no-plots         # skip post-run plot generation
```

Output CSVs are saved to `~/prob_ros_ws/logs/`:

| File | Contents |
|---|---|
| `filter_data_with_resampling.csv` | Baseline PF run (resampling enabled) |
| `filter_data_without_resampling.csv` | Special Task run (resampling disabled) |
| `filter_data_<timestamp>.csv` | Timestamped copy of every run |

---

### Manual launch (single run)

```bash
source /opt/ros/jazzy/setup.bash
source ~/prob_ros_ws/install/setup.bash
ros2 launch probabilistic_robot_lab filters.launch.py
```

Everything starts in the correct order automatically:

| Delay | What starts |
|---|---|
| 0 s | Gazebo + Nav2 + AMCL |
| 5 s | KF + EKF nodes + RViz + Data Logger |
| 10 s | AutoNav — sets initial pose, drives 8-waypoint route |

RViz opens with pre-configured displays for `/pose_kf`, `/pose_ekf`, `/pose_pf`, and the particle cloud.

**With Particle Filter:**

```bash
ros2 launch probabilistic_robot_lab filters.launch.py pf:=true
```

**Special Task — Particle Degeneration (no resampling):**

```bash
ros2 launch probabilistic_robot_lab filters.launch.py pf:=true resampling:=false
```

**Noise Experiments (mandatory — vary Q and R):**

```bash
# High process noise (lower trust in motion model)
ros2 launch probabilistic_robot_lab filters.launch.py sigma_process:=0.1

# High measurement noise (lower trust in odometry)
ros2 launch probabilistic_robot_lab filters.launch.py sigma_meas:=0.5

# Both varied simultaneously
ros2 launch probabilistic_robot_lab filters.launch.py sigma_process:=0.1 sigma_meas:=0.5
```

Full Q/R sensitivity study (4 runs: low/high σ_process, low/high σ_meas) with exact
commands, result CSVs, and a summary table is documented in **`EXPERIMENT_LOG.md`**.
Comparison figures: `scripts/qr_study_grid.pdf` (trajectory/RMSE/covariance per run)
and `scripts/qr_study_bar.pdf` (RMSE bar chart), generated via `scripts/plot_qr_study.py`.

---

### Navigation route

The robot spawns at map position `(-1.96, -0.5, 0°)` (= odom origin).  
AutoNav visits 8 waypoints across 4 unique positions, covering a ~3.6 m diameter loop:

```
WP1/6  ( 0.0,  1.8) — north
WP2/5/8 (-1.8,  0.0) — west
WP3/7  ( 0.0, -1.8) — south
WP4    ( 1.8,  0.0) — east
```

To change the route, edit `INITIAL_POSE` and `WAYPOINTS` in `scripts/auto_nav.py` and keep `WAYPOINTS` in `scripts/plot_results.py` in sync.

---

### Plotting results

**All paper plots in one command:**

```bash
cd ~/prob_ros_ws/src/probabilistic_robot_lab
./scripts/run_plots.sh
# auto-picks the two most recent CSVs from ~/prob_ros_ws/logs/
```

Or with explicit files:

```bash
./scripts/run_plots.sh \
    ~/prob_ros_ws/logs/filter_data_with_resampling.csv \
    ~/prob_ros_ws/logs/filter_data_without_resampling.csv
```

**Individual scripts:**

```bash
# Filter comparison (Trajectory, Error, RMSE, Covariance Trace, N_eff)
python3 scripts/plot_results.py ~/prob_ros_ws/logs/filter_data_with_resampling.csv

# Special Task: N_eff comparison with vs. without resampling
python3 scripts/plot_n_eff.py \
    ~/prob_ros_ws/logs/filter_data_with_resampling.csv \
    ~/prob_ros_ws/logs/filter_data_without_resampling.csv

# Landmark range visualization (for landmark tuning)
python3 scripts/landmark_viz.py
```

---

## Nodes and Scripts

### ROS 2 Nodes

| Node | Executable | Subscribes | Publishes |
|---|---|---|---|
| Kalman Filter | `kf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_kf` |
| Extended KF | `ekf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_ekf` |
| Particle Filter | `pf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_pf`, `/pf_particles`, `/n_eff` |
| Data Logger | `data_logger` | `/odom`, `/pose_kf`, `/pose_ekf`, `/pose_pf`, `/n_eff` | CSV file |
| AutoNav | `auto_nav` | — | `/initialpose`, Nav2 goals |

**State vector (all filters):** `mu = [x, y, θ]ᵀ` — reference frame: `odom`

### Python Scripts

| Script | Purpose |
|---|---|
| `scripts/run_experiments.sh` | Full automated pipeline: launch → navigate → save CSV → plot |
| `scripts/run_plots.sh` | Generate all paper plots from existing CSVs |
| `scripts/plot_results.py` | 5-panel comparison figure (trajectory, error, RMSE, covariance, N_eff) |
| `scripts/plot_n_eff.py` | Special Task: N_eff and RMSE comparison with vs. without resampling |
| `scripts/plot_qr_study.py` | Q/R sensitivity study: 5-run comparison grid + RMSE bar chart |
| `scripts/landmark_viz.py` | Visualize landmark positions and per-filter range residuals |
| `scripts/data_logger.py` | ROS 2 node: log filter outputs to timestamped CSV |
| `scripts/auto_nav.py` | ROS 2 node: set AMCL initial pose and drive fixed waypoint route |

---

## Filter Overview

### Kalman Filter (KF)

Prediction uses a **linearised** B matrix (state-dependent cos/sin of θ — accepted approximation).  
Correction step 1: identity measurement matrix `C = I₃` against `/odom`.  
Correction step 2: linearised Jacobian `C_lm` for each visible landmark range.

Reference: Thrun (2006) Table 3.1

### Extended Kalman Filter (EKF)

Prediction uses the **exact nonlinear** motion function `g(u, μ)` and its Jacobian `G = ∂g/∂μ`.  
Correction step 1: `H = I₃` against `/odom`.  
Correction step 2: Jacobian `H_j = ∂h/∂μ` for each visible landmark.  
Key advantage over KF: no linearisation error in the prediction step.

Reference: Thrun (2006) Table 3.3 / Table 7.2

### Particle Filter (PF)

Monte Carlo Localization with systematic resampling.  
Motion model: `x_t^m ~ p(x_t | u_t, x_{t-1}^m)` — Gaussian noise on v and ω.  
Measurement weights: Gaussian likelihood on odometry + landmark range residuals.  
Resampling: systematic resampling triggered when `N_eff / N < 0.5`.

Reference: Thrun (2006) Table 4.3 / Table 4.4

---

## Landmarks

All three filters share the same landmark map for a fair comparison.  
Defined in `include/probabilistic_robot_lab/landmark_map.hpp`.

**To update landmark positions:**

1. Start the simulation and open RViz.
2. Click **Publish Point** (toolbar) on the visible pillar obstacles.
3. Read the coordinates: `ros2 topic echo /clicked_point --once`
4. Update `LANDMARK_MAP` in `landmark_map.hpp` and run `colcon build`.
5. Update the matching list in `scripts/landmark_viz.py`.

Run `python3 scripts/landmark_viz.py` to visualise range residuals and verify the gate threshold.

---

## Special Task (ID 2510331009) — Particle Degeneration

Without resampling, low-weight particles are never replaced.  
Over time all probability mass concentrates on a single particle — **weight collapse**.

**Effective Sample Size:**

```
N_eff = 1 / Σ wᵢ²     ∈ [1, N]
```

- `N_eff = N` — uniform weights, maximum diversity (ideal)
- `N_eff = 1` — single particle carries all weight (full degeneration)

**Experiment:**

```bash
cd ~/prob_ros_ws/src/probabilistic_robot_lab
./scripts/run_experiments.sh   # runs both, plots comparison automatically
```

**Observed result:** N_eff/N collapses from ~1.0 to ~0.0 within ~20 s without resampling and remains at numerical zero for the rest of the run. RMSE stays comparable short-term because the surviving particle continues to integrate odometry correctly — but the filter loses its uncertainty representation entirely and cannot recover from drift.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `sigma_process` | `0.01` | Process noise std dev → scales R (motion uncertainty) |
| `sigma_meas` | `0.10` | Measurement noise std dev → scales Q (sensor uncertainty) |
| `num_particles` | `500` | Number of PF particles |
| `resampling` | `true` | Enable systematic resampling (`false` = Special Task) |

---

## Status

### Implementation

| | KF | EKF | PF |
|---|---|---|---|
| Prediction step | ✅ | ✅ | ✅ |
| Correction step (odom) | ✅ | ✅ | ✅ |
| Correction step (landmarks) | ✅ | ✅ | ✅ |
| Full 3×3 covariance published | ✅ | ✅ | ✅ |
| RViz visualization | ✅ | ✅ | ✅ |
| Timestamp check (stale odom) | ✅ | ✅ | — |
| Tunable Q/R parameters | ✅ | ✅ | ✅ |

### Experiments

| Experiment | Status |
|---|---|
| Ground truth RMSE vs `/odom` | ✅ `plot_results.py` |
| Covariance trace visualization | ✅ `plot_results.py` |
| Landmark detection (shared map) | ✅ `landmark_viz.py` |
| Special Task: N_eff with vs. without resampling | ✅ `plot_n_eff.py` |
| Process noise (σ_process) variation | ✅ `EXPERIMENT_LOG.md` |
| Measurement noise (σ_meas) variation | ✅ `EXPERIMENT_LOG.md` |
| Runtime comparison KF / EKF / PF | 🔲 |

### Submission

| | Status |
|---|---|
| GitHub repo + README | ✅ |
| Paper (documentation) | 🔲 |
| PowerPoint presentation | 🔲 |

---

## References

- Thrun, S., Burgard, W., Fox, D. (2006). *Probabilistic Robotics*. MIT Press.
  - Table 3.1 — Kalman Filter algorithm
  - Table 3.3 — Extended Kalman Filter algorithm
  - Table 4.3 — Particle Filter (MCL)
  - Table 4.4 — Systematic Resampling
  - Table 7.2 — EKF Localization with known landmarks
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy)