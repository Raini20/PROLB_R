#!/usr/bin/env python3
"""
Aggregate Results — Multi-Run Experiment Analysis
==================================================
Reads all run*.csv files produced by run_multi_experiments.sh, computes
mean ± std RMSE per filter and condition, flags statistical outliers,
generates grouped box plots, and appends a structured summary section to
EXPERIMENT_LOG.md.

Usage:
    # Analyse a specific run directory
    python3 scripts/aggregate_results.py ~/prob_ros_ws/logs/multi_20260623_120000

    # Auto-detect the most recent multi_* directory
    python3 scripts/aggregate_results.py

Output (all written into <multi_run_dir>/):
    summary.md          Human-readable report with per-run tables and
                        cross-condition summary (appended to EXPERIMENT_LOG.md)
    boxplot_rmse.pdf    Grouped box plots: one group per condition, one box
                        per filter (KF / EKF / PF)

Dependencies:
    pip install matplotlib pandas numpy --break-system-packages
"""

import sys
import os
import glob
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Patch

matplotlib.rcParams.update({
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'figure.dpi': 150,
})

# ── Paths ──────────────────────────────────────────────────────────────────────
LOG_DIR = Path(os.path.expanduser('~/prob_ros_ws/logs'))
EXPERIMENT_LOG = Path(
    os.path.expanduser('~/prob_ros_ws/src/probabilistic_robot_lab/EXPERIMENT_LOG.md')
)

# ── Constants ──────────────────────────────────────────────────────────────────
# Human-readable labels for each condition (must match CONDITION_ARGS keys in
# run_multi_experiments.sh).
CONDITION_LABELS: dict[str, str] = {
    'baseline':      'Baseline (σ_p=0.01, σ_m=0.10, resample ON)',
    'no_resample':   'No Resampling — Special Task (σ_p=0.01, σ_m=0.10)',
    'qr_proc_low':   'Low Process Noise (σ_p=0.001, σ_m=0.10)',
    'qr_proc_high':  'High Process Noise (σ_p=0.10,  σ_m=0.10)',
    'qr_meas_low':   'Low Measurement Noise (σ_p=0.01, σ_m=0.01)',
    'qr_meas_high':  'High Measurement Noise (σ_p=0.01, σ_m=0.50)',
}

# Canonical display order (conditions not in this list are shown alphabetically
# after the canonical ones)
CONDITION_ORDER = [
    'baseline', 'no_resample',
    'qr_proc_low', 'qr_proc_high',
    'qr_meas_low',  'qr_meas_high',
]

FILTERS        = ['kf', 'ekf', 'pf']
FILTER_LABELS  = {'kf': 'KF', 'ekf': 'EKF', 'pf': 'PF'}
FILTER_COLORS  = {'kf': '#1f77b4', 'ekf': '#d62728', 'pf': '#2ca02c'}

# Values more than OUTLIER_SIGMA standard deviations from the mean are flagged.
OUTLIER_SIGMA = 2.0


# ── RMSE helpers ──────────────────────────────────────────────────────────────

def _pos_error(df: pd.DataFrame, prefix: str) -> pd.Series:
    """Euclidean position error between a filter estimate and /odom."""
    dx = df[f'{prefix}_x'] - df['odom_x']
    dy = df[f'{prefix}_y'] - df['odom_y']
    return np.sqrt(dx**2 + dy**2)


def _cumulative_rmse(err: pd.Series) -> pd.Series:
    """Running cumulative RMSE from a position-error series."""
    sq = err.fillna(0.0) ** 2
    return np.sqrt(np.cumsum(sq) / (np.arange(len(sq)) + 1))


def final_rmse(df: pd.DataFrame, prefix: str) -> float:
    """
    Return the final cumulative RMSE for a given filter prefix.
    Returns NaN if the filter produced no data in this run.
    """
    col = f'{prefix}_x'
    if col not in df.columns or df[col].isna().all():
        return float('nan')
    try:
        err = _pos_error(df, prefix)
        return float(_cumulative_rmse(err).iloc[-1])
    except Exception:
        return float('nan')


# ── Data loading ──────────────────────────────────────────────────────────────

def load_run(csv_path: Path) -> dict[str, float]:
    """Load one run CSV and return {filter_name: final_rmse}."""
    df = pd.read_csv(csv_path)
    return {f: final_rmse(df, f) for f in FILTERS}


def load_condition(cond_dir: Path) -> tuple[list[dict], list[Path]]:
    """
    Load all run*.csv files in a condition directory.

    Returns:
        runs  — list of {filter: rmse} dicts, one per run
        paths — list of Path objects in the same order
    """
    csv_paths = sorted(cond_dir.glob('run*.csv'))
    runs: list[dict] = []
    paths: list[Path] = []
    for p in csv_paths:
        try:
            runs.append(load_run(p))
            paths.append(p)
        except Exception as exc:
            print(f'  [WARN] Could not load {p.name}: {exc}')
    return runs, paths


# ── Statistics ────────────────────────────────────────────────────────────────

def compute_stats(values: list[float]) -> dict:
    """
    Compute descriptive statistics for a list of per-run RMSE values.

    Returns a dict with keys:
        mean, std, min, max, n, outlier_indices
    where outlier_indices lists run indices that deviate by > OUTLIER_SIGMA * std.
    """
    valid = np.array([v for v in values if not np.isnan(v)])
    if valid.size == 0:
        return {
            'mean': np.nan, 'std': np.nan,
            'min': np.nan,  'max': np.nan,
            'n': 0, 'outlier_indices': [],
        }
    mean = float(valid.mean())
    std  = float(valid.std())
    outlier_indices = [
        i for i, v in enumerate(values)
        if not np.isnan(v) and abs(v - mean) > OUTLIER_SIGMA * std
    ]
    return {
        'mean': mean, 'std': std,
        'min': float(valid.min()), 'max': float(valid.max()),
        'n': int(valid.size), 'outlier_indices': outlier_indices,
    }


# ── Main analysis ─────────────────────────────────────────────────────────────

def analyze(multi_dir: Path) -> tuple[dict, dict]:
    """
    Walk through all condition subdirectories, load runs, compute stats.

    Returns:
        results — dict[condition] -> dict[filter] -> {runs, stats, paths}
        meta    — dict from run_meta.txt (may be empty if file absent)
    """
    # Load optional metadata written by run_multi_experiments.sh
    meta: dict[str, str] = {}
    meta_path = multi_dir / 'run_meta.txt'
    if meta_path.exists():
        for line in meta_path.read_text(encoding='utf-8').splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                meta[k.strip()] = v.strip()

    # Collect conditions in canonical order, then any extras alphabetically
    found_dirs = {d.name: d for d in sorted(multi_dir.iterdir()) if d.is_dir()}
    ordered_keys = [k for k in CONDITION_ORDER if k in found_dirs]
    ordered_keys += sorted(k for k in found_dirs if k not in ordered_keys)

    results: dict = {}
    for condition in ordered_keys:
        cond_dir = found_dirs[condition]
        runs, paths = load_condition(cond_dir)
        if not runs:
            continue
        results[condition] = {}
        for f in FILTERS:
            values = [r[f] for r in runs]
            results[condition][f] = {
                'runs':  values,
                'stats': compute_stats(values),
                'paths': paths,
            }
        n = len(runs)
        print(f'  {condition}: {n} run(s) loaded')

    return results, meta


# ── Box plot ──────────────────────────────────────────────────────────────────

def generate_boxplot(results: dict, out_path: Path) -> None:
    """
    Grouped box plot: one group per condition, one box per filter.
    Outliers are shown as × markers.
    """
    conditions = list(results.keys())
    if not conditions:
        print('[WARN] No conditions found — skipping box plot.')
        return

    n_cond = len(conditions)
    fig_w  = max(10, n_cond * 3.0)
    fig, ax = plt.subplots(figsize=(fig_w, 6))
    fig.suptitle(
        'RMSE Distribution across Runs — Multi-Run Experiment\n'
        f'Box: Q1–Q3, whiskers: 1.5×IQR, × = outlier  '
        f'(>{OUTLIER_SIGMA}σ from mean flagged in tables)',
        fontsize=10, fontweight='bold',
    )

    positions:    list[float] = []
    box_data:     list[list]  = []
    colors_list:  list[str]   = []
    xtick_pos:    list[float] = []
    xtick_labels: list[str]   = []

    GROUP_WIDTH  = len(FILTERS)       # boxes per group
    GROUP_GAP    = 1                  # extra space between groups
    group_stride = GROUP_WIDTH + GROUP_GAP

    for g_idx, condition in enumerate(conditions):
        base = g_idx * group_stride
        xtick_pos.append(base + (GROUP_WIDTH - 1) / 2.0)
        short_label = CONDITION_LABELS.get(condition, condition)
        # Wrap long labels at ' (' to keep the plot readable
        xtick_labels.append(short_label.replace(' (', '\n('))

        for f_idx, f in enumerate(FILTERS):
            pos = base + f_idx
            values = [v for v in results[condition][f]['runs'] if not np.isnan(v)]
            positions.append(pos)
            box_data.append(values if values else [np.nan])
            colors_list.append(FILTER_COLORS[f])

    bp = ax.boxplot(
        box_data,
        positions=positions,
        widths=0.6,
        patch_artist=True,
        medianprops=dict(color='black', linewidth=2.0),
        flierprops=dict(marker='x', markersize=7, markeredgewidth=1.5,
                        linestyle='none', markeredgecolor='#555555'),
        notch=False,
        manage_ticks=False,
    )
    for patch, color in zip(bp['boxes'], colors_list):
        patch.set_facecolor(color)
        patch.set_alpha(0.65)

    # Legend for filter colours
    legend_handles = [
        Patch(facecolor=FILTER_COLORS[f], alpha=0.65, label=FILTER_LABELS[f])
        for f in FILTERS
    ]
    ax.legend(handles=legend_handles, loc='upper right')

    ax.set_xticks(xtick_pos)
    ax.set_xticklabels(xtick_labels, fontsize=8)
    ax.set_ylabel('Final Cumulative RMSE [m]')
    ax.set_title('RMSE per Filter per Condition')
    ax.grid(True, axis='y', alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, bbox_inches='tight')
    print(f'Box plot saved → {out_path}')
    try:
        subprocess.Popen(['explorer.exe', str(out_path).replace('/', '\\')])
    except FileNotFoundError:
        pass   # non-WSL environment — ignore


# ── Markdown generation ───────────────────────────────────────────────────────

def _fmt_cell(stats: dict) -> str:
    """Format a single table cell as 'mean ± std' with optional outlier mark."""
    if np.isnan(stats['mean']):
        return 'n/a'
    mark = ' ⚠' if stats['outlier_indices'] else ''
    return f"{stats['mean']:.4f} ± {stats['std']:.4f}{mark}"


def _fmt_val(value: float, is_outlier: bool) -> str:
    """Format a single per-run value with optional outlier mark."""
    if np.isnan(value):
        return 'n/a'
    mark = ' ⚠' if is_outlier else ''
    return f'{value:.4f}{mark}'


def generate_markdown(results: dict, meta: dict, multi_dir: Path) -> str:
    """
    Build the markdown summary text that is appended to EXPERIMENT_LOG.md.
    """
    ts       = meta.get('timestamp', datetime.now().strftime('%Y%m%d_%H%M%S'))
    n_runs   = meta.get('runs', '?')
    conds    = meta.get('conditions', ' '.join(results.keys()))

    lines: list[str] = []

    lines.append('\n\n---\n\n')
    lines.append(f'## Multi-Run Results — {ts}\n\n')
    lines.append(f'**Runs per condition:** {n_runs}  \n')
    lines.append(f'**Conditions:** `{conds}`  \n')
    lines.append(f'**Outlier threshold:** mean ± {OUTLIER_SIGMA}σ (marked ⚠)  \n')
    lines.append(f'**Output directory:** `{multi_dir}`  \n')
    lines.append(f'**Box plot:** `{multi_dir}/boxplot_rmse.pdf`  \n\n')

    # ── Per-condition tables ──────────────────────────────────────────────────
    for condition, fdata in results.items():
        label = CONDITION_LABELS.get(condition, condition)
        lines.append(f'### {label}\n\n')

        n = max(len(d['runs']) for d in fdata.values())
        lines.append('| Run | KF RMSE [m] | EKF RMSE [m] | PF RMSE [m] |\n')
        lines.append('|----:|------------:|-------------:|------------:|\n')

        for i in range(n):
            cells = []
            for f in FILTERS:
                vals     = fdata[f]['runs']
                outliers = fdata[f]['stats']['outlier_indices']
                val      = vals[i] if i < len(vals) else float('nan')
                cells.append(_fmt_val(val, i in outliers))
            lines.append(f'| {i + 1} | {cells[0]} | {cells[1]} | {cells[2]} |\n')

        # Summary rows
        lines.append('| **Mean ± Std** |')
        for f in FILTERS:
            lines.append(f' **{_fmt_cell(fdata[f]["stats"])}** |')
        lines.append('\n')

        lines.append('| **Min / Max** |')
        for f in FILTERS:
            s = fdata[f]['stats']
            if np.isnan(s['min']):
                lines.append(' n/a |')
            else:
                lines.append(f' {s["min"]:.4f} / {s["max"]:.4f} |')
        lines.append('\n\n')

        # Outlier commentary
        for f in FILTERS:
            idx = fdata[f]['stats']['outlier_indices']
            if idx:
                run_nums = ', '.join(f'Run {i + 1}' for i in idx)
                lines.append(
                    f'> **{FILTER_LABELS[f]} outlier(s):** {run_nums} '
                    f'deviate more than {OUTLIER_SIGMA}σ from the mean.\n'
                )
        lines.append('\n')

    # ── Cross-condition summary table ─────────────────────────────────────────
    lines.append('### Cross-Condition Summary\n\n')
    lines.append(
        '| Condition | KF Mean ± Std [m] | EKF Mean ± Std [m] '
        '| PF Mean ± Std [m] | Best filter |\n'
    )
    lines.append(
        '|-----------|:-----------------:|:------------------:'
        '|:-----------------:|:-----------:|\n'
    )

    for condition, fdata in results.items():
        short = CONDITION_LABELS.get(condition, condition).split(' (')[0]
        means = {f: fdata[f]['stats']['mean'] for f in FILTERS}
        # Best filter = lowest mean RMSE (ignoring NaN)
        valid_means = {f: v for f, v in means.items() if not np.isnan(v)}
        best = min(valid_means, key=valid_means.get) if valid_means else '—'

        row = f'| {short} |'
        for f in FILTERS:
            row += f' {_fmt_cell(fdata[f]["stats"])} |'
        row += f' **{FILTER_LABELS.get(best, best)}** |\n'
        lines.append(row)

    lines.append('\n')
    lines.append(
        '> ⚠ = at least one run flagged as outlier.  \n'
        '> RMSE is measured against `/odom` (Gazebo ground truth).  \n'
        '> KF/EKF RMSE at σ_meas=0.01 collapses to ≈0.006 m due to '
        'circularity (filter trusts `/odom` unconditionally; RMSE also '
        'measured against `/odom`).\n\n'
    )

    return ''.join(lines)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    if len(sys.argv) >= 2:
        multi_dir = Path(sys.argv[1]).expanduser()
    else:
        candidates = sorted(LOG_DIR.glob('multi_*'), reverse=True)
        if not candidates:
            print('ERROR: No multi_* directory found in ~/prob_ros_ws/logs/')
            print('Usage: python3 scripts/aggregate_results.py <multi_run_dir>')
            sys.exit(1)
        multi_dir = candidates[0]
        print(f'Auto-loading most recent run directory: {multi_dir}')

    if not multi_dir.is_dir():
        print(f'ERROR: {multi_dir} is not a directory.')
        sys.exit(1)

    print(f'\nAnalysing: {multi_dir}')
    results, meta = analyze(multi_dir)

    if not results:
        print('ERROR: No run*.csv files found — nothing to aggregate.')
        sys.exit(1)

    # ── Box plot ──────────────────────────────────────────────────────────────
    boxplot_path = multi_dir / 'boxplot_rmse.pdf'
    generate_boxplot(results, boxplot_path)

    # ── Markdown summary ──────────────────────────────────────────────────────
    md_text = generate_markdown(results, meta, multi_dir)

    summary_path = multi_dir / 'summary.md'
    summary_path.write_text(md_text, encoding='utf-8')
    print(f'Summary saved  → {summary_path}')

    # ── Append to EXPERIMENT_LOG.md ───────────────────────────────────────────
    if EXPERIMENT_LOG.exists():
        with open(EXPERIMENT_LOG, 'a', encoding='utf-8') as fh:
            fh.write(md_text)
        print(f'Appended to    → {EXPERIMENT_LOG}')
    else:
        print(
            f'[WARN] EXPERIMENT_LOG.md not found at {EXPERIMENT_LOG}\n'
            f'       Only summary.md was written. Copy it manually if needed.'
        )

    print('\nDone.')


if __name__ == '__main__':
    main()
