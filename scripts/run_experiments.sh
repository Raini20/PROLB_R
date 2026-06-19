#!/usr/bin/env bash
# =============================================================================
# run_experiments.sh — Automated PF resampling comparison pipeline
# =============================================================================
#
# Launches the full simulation twice (PF with and without resampling), waits
# for AutoNav to finish each route automatically, then runs all plots.
# No Ctrl+C needed — everything shuts down and re-starts on its own.
#
# USAGE
#   ./scripts/run_experiments.sh [OPTIONS]
#
# OPTIONS
#   (none)                   Run both experiments, then plot  [default]
#   --with-resampling        Run only the resampling-ON experiment
#   --without-resampling     Run only the resampling-OFF experiment
#   --no-plots               Skip post-experiment plot generation
#   --timeout=N              Seconds to wait for nav completion  (default: 300)
#   --startup-wait=N         Seconds to wait for Nav2 startup    (default: 40)
#   -h, --help               Show this help and exit
#
# EXAMPLES
#   ./scripts/run_experiments.sh
#   ./scripts/run_experiments.sh --with-resampling
#   ./scripts/run_experiments.sh --without-resampling --no-plots
#   ./scripts/run_experiments.sh --timeout=600
#
# HOW IT WORKS
#   1. Launches:  ros2 launch probabilistic_robot_lab filters.launch.py pf:=true resampling:=<X>
#      This brings up Gazebo, Nav2, all filter nodes, RViz, DataLogger, and AutoNav.
#   2. Waits for AutoNav to write ~/prob_ros_ws/logs/.nav_done (sentinel file).
#   3. Sends SIGINT to the launch process group for a clean ROS2 shutdown.
#   4. Copies the freshly-written CSV as filter_data_<label>.csv for the plots.
#   5. Repeats for the second experiment (if requested).
#   6. Calls run_plots.sh with both CSVs for the paper figures.
# =============================================================================
set -eo pipefail   # NOTE: -u intentionally omitted here — ROS2 setup scripts
                   # reference unset variables internally, which would abort
                   # under -u. It is re-enabled below after sourcing.

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$HOME/prob_ros_ws"
LOGS_DIR="$WS_DIR/logs"
SENTINEL="$LOGS_DIR/.nav_done"

# ── Defaults ──────────────────────────────────────────────────────────────────
RUN_WITH=true
RUN_WITHOUT=true
PLOTS=true
TIMEOUT=300       # seconds before giving up on nav completion
STARTUP_WAIT=40   # seconds to wait for Nav2 / AMCL to be ready

# ── Argument parsing ──────────────────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        --with-resampling)    RUN_WITH=true;  RUN_WITHOUT=false ;;
        --without-resampling) RUN_WITH=false; RUN_WITHOUT=true  ;;
        --no-plots)           PLOTS=false ;;
        --timeout=*)          TIMEOUT="${arg#*=}" ;;
        --startup-wait=*)     STARTUP_WAIT="${arg#*=}" ;;
        -h|--help)
            sed -n '/^# USAGE/,/^# HOW/{ /^# HOW/q; p }' "$0"
            exit 0 ;;
        *)  echo "[ERROR] Unknown argument: $arg"; exit 1 ;;
    esac
done

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
head_()  { echo -e "\n${BOLD}${CYAN}$*${NC}"; }

# ── Source ROS2 ───────────────────────────────────────────────────────────────
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

# Re-enable unbound-variable check now that ROS2 setup scripts are done
set -u

mkdir -p "$LOGS_DIR"

# ── Cleanup helper ────────────────────────────────────────────────────────────
# Called on EXIT/INT/TERM to ensure the simulation is always stopped.
LAUNCH_PID=""

cleanup() {
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        info "Stopping simulation (PID $LAUNCH_PID)..."
        # Negative PID = entire process group → clean ROS2 teardown
        kill -SIGINT -"$LAUNCH_PID" 2>/dev/null \
            || kill -SIGINT "$LAUNCH_PID" 2>/dev/null \
            || true
        local waited=0
        while kill -0 "$LAUNCH_PID" 2>/dev/null && [[ $waited -lt 15 ]]; do
            sleep 1; ((waited++))
        done
        # Force-kill anything still alive
        kill -SIGKILL -"$LAUNCH_PID" 2>/dev/null || true
    fi
    LAUNCH_PID=""
    # Belt-and-braces: remove any lingering Gazebo / ROS processes
    pkill -f "gz sim"                2>/dev/null || true
    pkill -f "gzserver"              2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    sleep 2
}

trap cleanup EXIT INT TERM

# ── run_experiment ────────────────────────────────────────────────────────────
# $1 = human label  ("with_resampling" | "without_resampling")
# $2 = resampling   ("true"            | "false")
# Prints the path of the saved CSV to stdout (last line); all other output
# goes to stderr so it doesn't pollute the captured return value.
run_experiment() {
    local label="$1"
    local resampling="$2"

    head_ "════════════════════════════════════════════════════════" >&2
    head_ " EXPERIMENT: pf resampling=$resampling  [$label]"        >&2
    head_ "════════════════════════════════════════════════════════" >&2

    # Remove stale sentinel from a previous run
    rm -f "$SENTINEL"

    # ── Start the full stack ──────────────────────────────────────────────────
    info "Launching: ros2 launch ... pf:=true resampling:=$resampling" >&2
    # setsid creates a new session → LAUNCH_PID becomes the process-group leader
    setsid ros2 launch probabilistic_robot_lab filters.launch.py \
        pf:=true resampling:="$resampling" \
        >/dev/null 2>&1 &
    LAUNCH_PID=$!
    info "Launch PID: $LAUNCH_PID" >&2

    # ── Wait for Nav2 + AutoNav to initialise ─────────────────────────────────
    info "Waiting ${STARTUP_WAIT}s for Nav2 / AutoNav to initialise..." >&2
    sleep "$STARTUP_WAIT"

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        error "Launch process died during startup — aborting." >&2
        LAUNCH_PID=""
        return 1
    fi

    # ── Poll for the sentinel file ─────────────────────────────────────────────
    info "Waiting for AutoNav to finish (timeout: ${TIMEOUT}s)..." >&2
    local elapsed=0
    while [[ $elapsed -lt $TIMEOUT ]]; do
        if [[ -f "$SENTINEL" ]]; then
            info "AutoNav finished ✓  (after ${elapsed}s)" >&2
            break
        fi
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            warn "Launch process exited early." >&2
            break
        fi
        sleep 2; ((elapsed += 2))
    done

    if [[ $elapsed -ge $TIMEOUT && ! -f "$SENTINEL" ]]; then
        warn "Navigation timed out — using data collected so far." >&2
    fi

    # ── Save a labelled copy of the CSV ───────────────────────────────────────
    local csv_out=""
    local latest
    latest=$(ls -t "$LOGS_DIR"/filter_data_[0-9]*.csv 2>/dev/null | head -1 || true)
    if [[ -n "$latest" ]]; then
        csv_out="$LOGS_DIR/filter_data_${label}.csv"
        cp "$latest" "$csv_out"
        info "CSV saved → $(basename "$csv_out")" >&2
    else
        warn "No CSV found for this experiment." >&2
    fi

    # ── Tear down ─────────────────────────────────────────────────────────────
    info "Stopping simulation..." >&2
    cleanup
    info "Done. Waiting 5 s before next experiment..." >&2
    sleep 5

    # Return the CSV path to the caller (stdout only — no other output here)
    echo "${csv_out:-}"
}

# ── Main ──────────────────────────────────────────────────────────────────────
CSV_WITH=""
CSV_WITHOUT=""

if $RUN_WITH; then
    CSV_WITH=$(run_experiment "with_resampling" "true") || true
fi

if $RUN_WITHOUT; then
    CSV_WITHOUT=$(run_experiment "without_resampling" "false") || true
fi

# ── Plots ─────────────────────────────────────────────────────────────────────
if $PLOTS; then
    head_ "════════════════════════════════════════════════════════"
    head_ " PLOTTING"
    head_ "════════════════════════════════════════════════════════"

    PLOTS_SH="$SCRIPT_DIR/run_plots.sh"
    if [[ ! -x "$PLOTS_SH" ]]; then
        warn "run_plots.sh not found / not executable at: $PLOTS_SH"
    else
        if [[ -n "$CSV_WITH" && -n "$CSV_WITHOUT" ]]; then
            bash "$PLOTS_SH" "$CSV_WITH" "$CSV_WITHOUT"
        elif [[ -n "$CSV_WITH" ]]; then
            bash "$PLOTS_SH" "$CSV_WITH"
        elif [[ -n "$CSV_WITHOUT" ]]; then
            bash "$PLOTS_SH" "$CSV_WITHOUT"
        else
            warn "No CSVs available — skipping plots."
        fi
    fi
fi

head_ "════════════════════════════════════════════════════════"
head_ " ALL DONE"
head_ "════════════════════════════════════════════════════════"
info "Logs: $LOGS_DIR"