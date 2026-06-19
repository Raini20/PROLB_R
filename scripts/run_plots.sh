#!/usr/bin/env bash
# run_plots.sh — one command to generate all paper plots
# Usage:
#   ./run_plots.sh                    # uses the 2 most recent CSVs
#   ./run_plots.sh file1.csv          # plot_results on file1, N_eff single
#   ./run_plots.sh file1.csv file2.csv # plot_results on file1, N_eff compares both

set -euo pipefail

LOGS="$HOME/prob_ros_ws/logs"
SCRIPTS="$HOME/prob_ros_ws/src/probabilistic_robot_lab/scripts"

# ── Resolve CSV arguments ────────────────────────────────────────────────────
if [ $# -ge 1 ]; then
    CSV1="$1"
else
    # Auto-pick: newest CSV in logs/
    CSV1=$(ls -t "$LOGS"/filter_data_*.csv 2>/dev/null | head -1 || true)
    if [ -z "$CSV1" ]; then
        echo "ERROR: No filter_data_*.csv found in $LOGS"
        exit 1
    fi
    echo "Auto-selected: $(basename "$CSV1")"
fi

if [ $# -ge 2 ]; then
    CSV2="$2"
else
    # Auto-pick: second newest CSV (for N_eff comparison)
    CSV2=$(ls -t "$LOGS"/filter_data_*.csv 2>/dev/null | sed -n '2p' || true)
fi

# ── Step 1: General comparison plot ─────────────────────────────────────────
echo ""
echo "━━━ plot_results.py ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
python3 "$SCRIPTS/plot_results.py" "$CSV1"

# ── Step 2: N_eff / Particle Degeneration plot ───────────────────────────────
echo ""
echo "━━━ plot_n_eff.py ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -n "$CSV2" ] && [ "$CSV2" != "$CSV1" ]; then
    echo "Comparing:"
    echo "  WITH resampling : $(basename "$CSV1")"
    echo "  NO   resampling : $(basename "$CSV2")"
    python3 "$SCRIPTS/plot_n_eff.py" "$CSV1" "$CSV2"
else
    echo "Only 1 CSV found — showing single-file N_eff plot"
    python3 "$SCRIPTS/plot_n_eff.py" "$CSV1"
fi

echo ""
echo "Done. PDFs opened in background."
