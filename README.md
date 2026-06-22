# Probabilistic Robot Lab (PROLB_R)

ROS 2 implementation of **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Particle Filter (PF)** for mobile robot localization on a TurtleBot3 Waffle simulation.

**Course:** Probabilistic Robotics (PRO), FH Technikum Wien  
**Special Task (2510331009):** Disable PF resampling and analyze particle degeneration via N_eff.  
**Algorithm reference:** Thrun, Burgard & Fox — *Probabilistic Robotics* (2006)

All experiment commands, raw result locations, and the full Q/R sensitivity study are in **[`EXPERIMENT_LOG.md`](EXPERIMENT_LOG.md)**.

---

## Environment

| | |
|---|---|
| OS | Ubuntu 24.04 (WSL2 / native) |
| ROS 2 | Jazzy |
| Simulation | TurtleBot3 Waffle + Nav2 + Gazebo |
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
./scripts/run_experiments.sh                       # both runs, then plot
./scripts/run_experiments.sh --with-resampling     # resampling-ON run only
./scripts/run_experiments.sh --without-resampling  # resampling-OFF run only
./scripts/run_experiments.sh --timeout=600         # extend nav timeout
./scripts/run_experiments.sh --no-plots            # skip post-run plotting
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
ros2 launch probabilistic_robot_lab filters.launch.py
```

Everything starts in the correct order automatically:

| Delay | What starts |
|---|---|
| 0 s | Gazebo + Nav2 + AMCL |
| 5 s | KF + EKF + PF nodes, RViz, Data Logger |
| 10 s | AutoNav — sets initial pose, drives 8-waypoint route |

**Special Task — no resampling:**

```bash
ros2 launch probabilistic_robot_lab filters.launch.py resampling:=false
```

**Noise experiments:**

```bash
ros2 launch probabilistic_robot_lab filters.launch.py sigma_process:=0.1 sigma_meas:=0.5
```

---

### Navigation route

The robot spawns at `(-1.96, -0.5, 0°)` in the odom frame and visits 8 waypoints:

```
WP1/6   ( 0.0,  1.8)  — north
WP2/5/8 (-1.8,  0.0)  — west  (passes closest to the anchor corner)
WP3/7   ( 0.0, -1.8)  — south
WP4     ( 1.8,  0.0)  — east
```

To change the route, edit `INITIAL_POSE` and `WAYPOINTS` in `scripts/auto_nav.py`, and keep `WAYPOINTS` in `scripts/plot_results.py` in sync.

---

### Plotting results

```bash
# All plots from the two canonical CSVs:
./scripts/run_plots.sh

# Or with explicit files:
./scripts/run_plots.sh \
    ~/prob_ros_ws/logs/filter_data_with_resampling.csv \
    ~/prob_ros_ws/logs/filter_data_without_resampling.csv

# Filter comparison only (trajectory, error, RMSE, covariance, N_eff):
python3 scripts/plot_results.py ~/prob_ros_ws/logs/filter_data_with_resampling.csv

# Special Task — N_eff with vs. without resampling:
python3 scripts/plot_n_eff.py \
    ~/prob_ros_ws/logs/filter_data_with_resampling.csv \
    ~/prob_ros_ws/logs/filter_data_without_resampling.csv
```

The comparison plot shades time windows where the **combined anchor feature** (pillar + corner walls) was simultaneously detected and corrections were applied.

---

## Nodes and Scripts

### ROS 2 Nodes

| Node | Executable | Subscribes | Publishes |
|---|---|---|---|
| Kalman Filter | `kf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_kf`, `/kf_markers` |
| Extended KF | `ekf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_ekf`, `/ekf_markers`, `/ekf_landmark_count`, `/ekf_wall_count` |
| Particle Filter | `pf_node` | `/cmd_vel`, `/odom`, `/scan` | `/pose_pf`, `/pf_particles`, `/n_eff`, `/pf_markers` |
| Data Logger | `data_logger` | `/odom`, `/pose_kf`, `/pose_ekf`, `/pose_pf`, `/n_eff`, `/ekf_landmark_count`, `/ekf_wall_count` | CSV file |
| AutoNav | `auto_nav` | — | `/initialpose`, Nav2 goals |

**State vector (all filters):** `mu = [x, y, θ]ᵀ` — reference frame: `odom`

### Key Header Files

| Header | Purpose |
|---|---|
| `landmark_map.hpp` | `LANDMARK_MAP` — the single anchor pillar LM_ML |
| `landmark_detector.hpp` | `detectCylinders()` + `associateLandmarks()` — pose-independent cylinder detection via beam clustering |
| `wall_detector.hpp` | `extractWalls()` + `associateWalls()` — RANSAC line extraction in Hesse normal form |
| `line_ransac.hpp` | Pure-geometry sequential RANSAC with TLS refinement (no ROS dependencies) |
| `anchor.hpp` | `anchor_detected()` — gate function requiring both LM_ML and corner walls simultaneously |
| `detection_markers.hpp` | `buildDetectionMarkers()` — RViz MarkerArray for all three filters (farbkodiert per filter) |

### Python Scripts

| Script | Purpose |
|---|---|
| `scripts/run_experiments.sh` | Full automated pipeline: launch → navigate → save CSV → plot |
| `scripts/run_plots.sh` | Generate all paper plots from existing CSVs |
| `scripts/plot_results.py` | 5-panel comparison figure (trajectory, error, RMSE, covariance, N_eff) |
| `scripts/plot_n_eff.py` | Special Task: N_eff and RMSE comparison with vs. without resampling |
| `scripts/plot_qr_study.py` | Q/R sensitivity study: 5-run comparison grid + RMSE bar chart |
| `scripts/data_logger.py` | ROS 2 node: log filter outputs to timestamped CSV |
| `scripts/auto_nav.py` | ROS 2 node: set AMCL initial pose and drive fixed waypoint route |

---

## Filter Overview

### Kalman Filter (KF)

Linear Gaussian filter. Prediction uses a linearised B matrix with pose-dependent cos/sin terms (accepted approximation — flagged as a known limitation vs. EKF's cleaner Jacobian formulation). Correction 1: identity measurement against `/odom`. Correction 2+3: range Jacobian `C_lm` and linear wall Jacobian `C_w` — both applied only when the combined anchor feature is active.

Reference: Thrun (2006) Table 3.1

### Extended Kalman Filter (EKF)

Nonlinear extension with Jacobian linearisation. Prediction uses the exact nonlinear motion function `g(u, μ)` and its Jacobian `G = ∂g/∂μ`. Correction 2: landmark range Jacobian `H_j = ∂h/∂μ`. Correction 3: wall measurement model is linear in the pose — the KF and EKF wall updates are therefore identical. Anchor gate applied to both corrections.

Reference: Thrun (2006) Table 3.3 / Table 7.2

### Particle Filter (PF)

Monte Carlo Localization with systematic resampling. Motion model: Gaussian noise on `v` and `ω`. Measurement weights: Gaussian likelihood on odometry + landmark range + wall line residuals. Wall and landmark likelihoods applied only when `anchor_detected()` is true. Resampling triggered when `N_eff / N < 0.5`.

Reference: Thrun (2006) Table 4.3 / Table 4.4

---

## Landmark and Anchor Detection

### The Symmetry Problem

The arena contains 9 identical cylindrical pillars in a 3×3 grid at 1.1 m spacing. Any single pillar is ambiguous from the laser scan alone — the local feature constellation is translationally symmetric, so association requires the current pose estimate as a prior.

### The Combined Anchor

Two features are combined to form the unambiguous **left-corner anchor**:

| Feature | Type | Description |
|---|---|---|
| `LM_ML` | Cylinder at (0.9175, 0.4675) | Middle-left pillar |
| `W_NW` | Wall α=+2.61 rad, ρ=0.95 m | Upper wall of left corner |
| `W_SW` | Wall α=−2.62 rad, ρ=0.41 m | Lower wall of left corner |

There is exactly one location in the arena where all three are co-visible. Neither the pillar alone (pose-dependent association among 9 identical pillars) nor the wall alone (Hesse parameters are associated via the current pose estimate) is unambiguous in isolation.

**Gate:** All filter corrections (landmark range + wall line) fire only when `anchor_detected()` returns true — i.e., when LM_ML and at least one corner wall are simultaneously matched.

### Wall Detection (RANSAC)

Implemented in `line_ransac.hpp` (pure geometry) and `wall_detector.hpp` (ROS wrapper). Lines are extracted from the laser scan in Hesse normal form `(α, ρ)` via sequential RANSAC with TLS refinement, then matched against the known `WALL_MAP` using angle and distance gates.

The wall measurement model `h(x) = [α − θ, ρ − (x cos α + y sin α)]ᵀ` is **linear in the pose**, so the KF and EKF wall corrections are exact (no additional linearisation error).

### Updating Landmark Positions

1. Start the simulation and open RViz (Fixed Frame = `odom`).
2. Click **Publish Point** on the pillar surface.
3. Read coordinates: `ros2 topic echo /clicked_point --field point`
4. Subtract the pillar radius (0.15 m) along the click direction to get the true center.
5. Update `LANDMARK_MAP` in `landmark_map.hpp` and run `colcon build`.

---

## RViz Visualization

All three filter nodes publish MarkerArrays showing the current detection state:

| Topic | Filter | Color |
|---|---|---|
| `/ekf_markers` | EKF | Red (`#d62728`) |
| `/kf_markers` | KF | Blue (`#1f77b4`) |
| `/pf_markers` | PF | Green (`#2ca02c`) |

Each marker set contains:
- **Pillar sphere** — grey when anchor is inactive, coloured when active
- **Pillar line** — robot → pillar, shown only when anchor is fully active
- **Wall segments** — grey when not matched, coloured when matched
- **Wall foot line** — robot → foot of perpendicular on wall, shown when matched

The sphere and line remain grey (and no line is drawn) when the wall is not co-detected, making it visually clear that the filter has not identified which specific pillar it sees.

---

## Special Task (ID 2510331009) — Particle Degeneration

Without resampling, particle weights collapse to a single particle within ~1 s.

**Effective Sample Size:**

```
N_eff = 1 / Σ wᵢ²     ∈ [1/N, 1]
```

- `N_eff = N` — uniform weights, maximum diversity (ideal)
- `N_eff = 1` — single particle carries all weight (full degeneration)

**Observed result:** N_eff/N collapses from ~1.0 to ~0.0 within 1 s without resampling and stays at numerical zero for the entire run. RMSE increases from 0.127 m (resampling ON) to 0.156 m (resampling OFF), with the degenerated filter unable to recover from drift.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `sigma_process` | `0.01` | Process noise std dev (motion uncertainty) |
| `sigma_meas` | `0.10` | Measurement noise std dev (sensor uncertainty) |
| `num_particles` | `500` | Number of PF particles |
| `resampling` | `true` | Enable systematic resampling (`false` = Special Task) |

---

## Known Limitations

**RMSE circularity:** All three filters use `/odom` as the motion model input (Correction 1). RMSE is also measured against `/odom`. This means the metric captures filter lag rather than absolute position accuracy — and anchor corrections that pull the estimate toward the true map position can appear as RMSE increases. In a real deployment with accumulating odometric drift, the anchor correction would produce clearly measurable improvements against an external reference.

**KF linearisation:** The B matrix in the KF prediction step contains pose-dependent cos/sin terms and is not strictly linear. The EKF handles this correctly via the Jacobian G. Both are flagged in the paper.

**Anchor coverage:** The route passes near the left-corner anchor (waypoint WP2/5/8 at ≈1.4 m from the corner). On this route, the anchor is visible ≈90% of the run time, so the gate rarely blocks corrections. A route that avoids the anchor corner would show a larger RMSE impact.

---

## Status

### Implementation

| | KF | EKF | PF |
|---|---|---|---|
| Prediction step | ✅ | ✅ | ✅ |
| Correction — odometry | ✅ | ✅ | ✅ |
| Correction — landmark range | ✅ | ✅ | ✅ |
| Correction — wall (RANSAC) | ✅ | ✅ | ✅ |
| Combined anchor gate | ✅ | ✅ | ✅ |
| Full 3×3 covariance published | ✅ | ✅ | ✅ |
| RViz detection markers | ✅ | ✅ | ✅ |
| Tunable Q/R via launch args | ✅ | ✅ | ✅ |

### Experiments

| Experiment | Status |
|---|---|
| RMSE vs `/odom` — all three filters | ✅ |
| Covariance trace visualization | ✅ |
| Anchor detection shading in plots | ✅ |
| Special Task: N_eff with vs. without resampling | ✅ |
| Q/R sensitivity study | ✅ `EXPERIMENT_LOG.md` |

### Submission

| | Status |
|---|---|
| GitHub repo + README | ✅ |
| Paper | 🔲 |
| Presentation | 🔲 |

---

## References

- Thrun, S., Burgard, W., Fox, D. (2006). *Probabilistic Robotics*. MIT Press.
  - Table 3.1 — Kalman Filter
  - Table 3.3 — Extended Kalman Filter
  - Table 4.3 — Particle Filter (MCL)
  - Table 4.4 — Systematic Resampling
  - Table 7.2 — EKF Localization with known landmarks
- [Nav2 Documentation](https://docs.nav2.org)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy)