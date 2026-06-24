#!/usr/bin/env bash
# =============================================================================
# run_multi_experiments.sh — Multi-run automated experiment pipeline
# =============================================================================
#
# Runs every experiment condition N times back-to-back without any manual
# interaction. After all runs, calls aggregate_results.py to compute
# mean ± std RMSE, flag outliers, generate box plots, and append a summary
# section to EXPERIMENT_LOG.md.
#
# USAGE
#   ./scripts/run_multi_experiments.sh [OPTIONS]
#
# OPTIONS
#   --runs=N            Runs per condition           (default: 5)
#   --conditions=LIST   Comma-separated subset:
#                         baseline, no_resample,
#                         qr_proc_low, qr_proc_high,
#                         qr_meas_low, qr_meas_high
#                       Use "all" for every condition (default: all)
#   --timeout=N         Nav completion timeout [s]   (default: 300)
#   --startup-wait=N    Nav2 startup wait [s]        (default: 40)
#   --no-plots          Skip aggregate_results.py after all runs
#   --out-dir=DIR       Custom output dir            (default: auto-timestamped)
#   -h, --help          Show this help and exit
#
# EXAMPLES
#   # Full study — 5 runs per condition (takes ~6 h)
#   ./scripts/run_multi_experiments.sh
#
#   # Quick test — 3 runs, baseline + special task only
#   ./scripts/run_multi_experiments.sh --runs=3 --conditions=baseline,no_resample
#
#   # Only regenerate aggregate plots from an existing run directory
#   python3 scripts/aggregate_results.py ~/prob_ros_ws/logs/multi_20260623_120000
#
# HOW IT WORKS
#   1. For each condition × run:
#      a. Launches: ros2 launch probabilistic_robot_lab filters.launch.py <args>
#         (Gazebo + Nav2 + filter nodes + DataLogger + AutoNav all start together)
#      b. Waits for AutoNav to write the .nav_done sentinel file.
#      c. Copies the freshly-written CSV to <out-dir>/<condition>/run<N>.csv
#      d. Sends SIGINT to the launch process group for a clean ROS 2 shutdown.
#   2. After all runs, calls aggregate_results.py <out-dir>.
# =============================================================================

# NOTE: -u is NOT set before sourcing ROS 2 setup scripts — they reference
# unset variables internally. It is re-enabled below after sourcing.
set -eo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$HOME/prob_ros_ws"
LOGS_DIR="$WS_DIR/logs"
SENTINEL="$LOGS_DIR/.nav_done"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

# ── Defaults ──────────────────────────────────────────────────────────────────
RUNS=5
CONDITIONS="all"
TIMEOUT=300
STARTUP_WAIT=40
PLOTS=true
OUT_DIR="$LOGS_DIR/multi_${TIMESTAMP}"

# ── Argument parsing ──────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --runs=*)          RUNS="${arg#*=}" ;;
        --conditions=*)    CONDITIONS="${arg#*=}" ;;
        --timeout=*)       TIMEOUT="${arg#*=}" ;;
        --startup-wait=*)  STARTUP_WAIT="${arg#*=}" ;;
        --no-plots)        PLOTS=false ;;
        --out-dir=*)       OUT_DIR="${arg#*=}" ;;
        -h|--help)
            sed -n '/^# USAGE/,/^# HOW/{ /^# HOW/q; p }' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "[ERROR] Unknown argument: $arg"; exit 1 ;;
    esac
done

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
head_() { echo -e "\n${BOLD}${CYAN}$*${NC}"; }

# ── Source ROS 2 ──────────────────────────────────────────────────────────────
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

# Re-enable unbound-variable check now that ROS 2 setup scripts are done
set -u

# ── Condition definitions ─────────────────────────────────────────────────────
# Each entry: "label" -> "ros2 launch arguments (space-separated)"
# Keeping sigma_process / sigma_meas explicit in every entry for reproducibility.
declare -A CONDITION_ARGS=(
    [baseline]="pf:=true resampling:=true  sigma_process:=0.01  sigma_meas:=0.10"
    [no_resample]="pf:=true resampling:=false sigma_process:=0.01  sigma_meas:=0.10"
    [qr_proc_low]="pf:=true resampling:=true  sigma_process:=0.001 sigma_meas:=0.10"
    [qr_proc_high]="pf:=true resampling:=true  sigma_process:=0.1   sigma_meas:=0.10"
    [qr_meas_low]="pf:=true resampling:=true  sigma_process:=0.01  sigma_meas:=0.01"
    [qr_meas_high]="pf:=true resampling:=true  sigma_process:=0.01  sigma_meas:=0.50"
)

# Canonical order (preserved in the summary report)
ALL_CONDITIONS=(baseline no_resample qr_proc_low qr_proc_high qr_meas_low qr_meas_high)

# Build the active condition list
if [[ "$CONDITIONS" == "all" ]]; then
    RUN_CONDITIONS=("${ALL_CONDITIONS[@]}")
else
    IFS=',' read -ra RUN_CONDITIONS <<< "$CONDITIONS"
fi

# ── Validate conditions early ─────────────────────────────────────────────────
for c in "${RUN_CONDITIONS[@]}"; do
    if [[ -z "${CONDITION_ARGS[$c]+_}" ]]; then
        error "Unknown condition: '$c'"
        echo "Valid conditions: ${!CONDITION_ARGS[*]}"
        exit 1
    fi
done

mkdir -p "$OUT_DIR"

LAUNCH_PID=""

# ── Cleanup ───────────────────────────────────────────────────────────────────
cleanup() {
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        info "Stopping simulation (PID $LAUNCH_PID)..."
        # Negative PID = entire process group → clean ROS 2 teardown
        kill -SIGINT -"$LAUNCH_PID" 2>/dev/null \
            || kill -SIGINT "$LAUNCH_PID" 2>/dev/null \
            || true
        local waited=0
        while kill -0 "$LAUNCH_PID" 2>/dev/null && [[ $waited -lt 15 ]]; do
            sleep 1
            waited=$((waited + 1))
        done
        kill -SIGKILL -"$LAUNCH_PID" 2>/dev/null || true
    fi
    LAUNCH_PID=""
    pkill -f "gz sim"                2>/dev/null || true
    pkill -f "gzserver"              2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    sleep 2
}

on_signal() {
    error "Interrupted — cleaning up..."
    cleanup
    exit 1
}
trap cleanup EXIT
trap on_signal INT TERM

# ── run_single ────────────────────────────────────────────────────────────────
# Runs one simulation, copies the resulting CSV, then shuts down cleanly.
# $1 = condition name (key in CONDITION_ARGS)
# $2 = run number (integer)
run_single() {
    local condition="$1"
    local run_num="$2"
    local launch_args="${CONDITION_ARGS[$condition]}"
    local cond_dir="$OUT_DIR/$condition"
    mkdir -p "$cond_dir"

    head_ "────────────────────────────────────────────────────────────────"
    head_ " CONDITION: $condition  |  Run $run_num / $RUNS"
    head_ " Args: $launch_args"
    head_ "────────────────────────────────────────────────────────────────"

    rm -f "$SENTINEL"

    local launch_log="$cond_dir/launch_run${run_num}.log"
    info "Launch log → $launch_log  (tail -f to follow)"

    # setsid creates a new session so kill -PGID can't accidentally reach us
    # SC2086: word-split of $launch_args is intentional here
    # shellcheck disable=SC2086
    setsid ros2 launch probabilistic_robot_lab filters.launch.py \
        $launch_args \
        >"$launch_log" 2>&1 &
    LAUNCH_PID=$!
    info "Launch PID: $LAUNCH_PID"

    # ── Wait for Nav2 / AutoNav to initialise ─────────────────────────────────
    info "Waiting ${STARTUP_WAIT}s for Nav2 / AutoNav startup..."
    sleep "$STARTUP_WAIT"

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        error "Launch process died during startup — aborting this run."
        LAUNCH_PID=""
        return 1
    fi

    # ── Poll for sentinel (AutoNav writes it when all waypoints are done) ─────
    info "Waiting for AutoNav to finish (timeout: ${TIMEOUT}s)..."
    local elapsed=0
    while [[ $elapsed -lt $TIMEOUT ]]; do
        if [[ -f "$SENTINEL" ]]; then
            info "AutoNav finished ✓  (after ${elapsed}s)"
            break
        fi
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            warn "Launch process exited early — using data collected so far."
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [[ $elapsed -ge $TIMEOUT && ! -f "$SENTINEL" ]]; then
        warn "Navigation timed out — using data collected so far."
    fi

    # ── Copy the most recent timestamped CSV to a stable path ─────────────────
    local latest
    latest=$(ls -t "$LOGS_DIR"/filter_data_[0-9]*.csv 2>/dev/null | head -1 || true)
    if [[ -n "$latest" ]]; then
        local dest="$cond_dir/run${run_num}.csv"
        cp "$latest" "$dest"
        info "CSV saved → $(realpath --relative-to="$HOME" "$dest")"
    else
        warn "No CSV found for condition=$condition run=$run_num — run may have failed."
    fi

    # ── Tear down ─────────────────────────────────────────────────────────────
    cleanup
    info "Cooling down 5 s before next run..."
    sleep 5
}

# ── Print experiment plan ─────────────────────────────────────────────────────
total_runs=$(( ${#RUN_CONDITIONS[@]} * RUNS ))

head_ "════════════════════════════════════════════════════════════════"
head_ " MULTI-RUN EXPERIMENT PIPELINE"
head_ " Conditions : ${RUN_CONDITIONS[*]}"
head_ " Runs each  : $RUNS"
head_ " Total runs : $total_runs"
head_ " Timeout    : ${TIMEOUT}s / Startup wait: ${STARTUP_WAIT}s"
head_ " Output dir : $OUT_DIR"
head_ "════════════════════════════════════════════════════════════════"

# Write metadata so aggregate_results.py can display it in the report
cat > "$OUT_DIR/run_meta.txt" << EOF
timestamp=${TIMESTAMP}
runs=${RUNS}
conditions=${RUN_CONDITIONS[*]}
timeout=${TIMEOUT}
startup_wait=${STARTUP_WAIT}
EOF

# ── Main loop ─────────────────────────────────────────────────────────────────
global_run=0
for condition in "${RUN_CONDITIONS[@]}"; do
    for run_num in $(seq 1 "$RUNS"); do
        global_run=$((global_run + 1))
        info "Progress: run $global_run / $total_runs total"
        run_single "$condition" "$run_num"
    done
    info "Condition '$condition' complete — 10 s inter-condition cooldown..."
    sleep 10
done

# ── Aggregate results ─────────────────────────────────────────────────────────
if $PLOTS; then
    head_ "════════════════════════════════════════════════════════════════"
    head_ " AGGREGATING RESULTS"
    head_ "════════════════════════════════════════════════════════════════"

    AGG_PY="$SCRIPT_DIR/aggregate_results.py"
    if [[ ! -f "$AGG_PY" ]]; then
        warn "aggregate_results.py not found at $AGG_PY — skipping aggregation."
        warn "Run manually: python3 scripts/aggregate_results.py \"$OUT_DIR\""
    else
        python3 "$AGG_PY" "$OUT_DIR"
    fi
fi

head_ "════════════════════════════════════════════════════════════════"
head_ " ALL DONE"
head_ "════════════════════════════════════════════════════════════════"
info "Results: $OUT_DIR"
info "Summary: $OUT_DIR/summary.md"
info "Plots:   $OUT_DIR/boxplot_rmse.pdf"
