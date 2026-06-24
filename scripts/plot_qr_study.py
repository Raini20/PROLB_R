#!/usr/bin/env python3
"""
plot_qr_study.py — Q/R Sensitivity Study Figures
==================================================
Generates a two-panel bar chart comparing KF / EKF / PF RMSE across
process-noise and measurement-noise conditions.

Data sources (in priority order):
  1. Multi-run directory (preferred — uses mean ± std from 5 runs per condition)
  2. Named single-run CSVs in ~/prob_ros_ws/logs/ (fallback)

Usage:
    # Auto-detect most recent multi_* directory + named CSVs as fallback
    python3 scripts/plot_qr_study.py

    # Explicit multi-run directory
    python3 scripts/plot_qr_study.py ~/prob_ros_ws/logs/multi_20260623_022008

Output:
    ~/prob_ros_ws/logs/qr_study.pdf   — 2-panel bar chart (process / meas noise)

Figure layout:
    Left panel  — Process noise sensitivity
                  Conditions: Baseline | σ_p=0.001 (low) | σ_p=0.10 (high)
    Right panel — Measurement noise sensitivity
                  Conditions: Baseline | σ_m=0.01 (low) | σ_m=0.50 (high)
    Both panels show grouped bars per filter (KF / EKF / PF) with ±1σ error bars.
    Single-run data is shown without error bars (n=1).
"""

import sys
import os
import glob
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

matplotlib.rcParams.update({
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'figure.dpi': 150,
})

# ── Paths ──────────────────────────────────────────────────────────────────────
LOG_DIR  = Path(os.path.expanduser('~/prob_ros_ws/logs'))
OUT_PATH = LOG_DIR / 'qr_study.pdf'

# ── Visual constants ───────────────────────────────────────────────────────────
FILTERS       = ['kf', 'ekf', 'pf']
FILTER_LABELS = {'kf': 'KF', 'ekf': 'EKF', 'pf': 'PF'}
FILTER_COLORS = {'kf': '#1f77b4', 'ekf': '#d62728', 'pf': '#2ca02c'}


# ── RMSE helpers ──────────────────────────────────────────────────────────────

def _pos_error(df: pd.DataFrame, prefix: str) -> pd.Series:
    dx = df[f'{prefix}_x'] - df['odom_x']
    dy = df[f'{prefix}_y'] - df['odom_y']
    return np.sqrt(dx**2 + dy**2)


def _cumulative_rmse(err: pd.Series) -> pd.Series:
    sq = err.fillna(0.0) ** 2
    return np.sqrt(np.cumsum(sq) / (np.arange(len(sq)) + 1))


def final_rmse(df: pd.DataFrame, prefix: str) -> float:
    """Final cumulative RMSE for one filter in one CSV. Returns NaN on failure."""
    col = f'{prefix}_x'
    if col not in df.columns or df[col].isna().all():
        return float('nan')
    try:
        return float(_cumulative_rmse(_pos_error(df, prefix)).iloc[-1])
    except Exception:
        return float('nan')


# ── Data loading ──────────────────────────────────────────────────────────────

def load_single_csv(path: Path) -> dict[str, float]:
    """Load one CSV → {filter: rmse}."""
    df = pd.read_csv(path)
    return {f: final_rmse(df, f) for f in FILTERS}


def load_multi_condition(cond_dir: Path) -> dict[str, dict]:
    """
    Load all run*.csv from a multi-run condition directory.
    Returns {filter: {mean, std, n}} or empty dict if no CSVs found.
    """
    csvs = sorted(cond_dir.glob('run*.csv'))
    if not csvs:
        return {}
    runs_per_filter: dict[str, list] = {f: [] for f in FILTERS}
    for csv in csvs:
        try:
            r = load_single_csv(csv)
            for f in FILTERS:
                if not np.isnan(r[f]):
                    runs_per_filter[f].append(r[f])
        except Exception as e:
            print(f'  [WARN] {csv.name}: {e}')
    result = {}
    for f in FILTERS:
        vals = runs_per_filter[f]
        if vals:
            arr = np.array(vals)
            result[f] = {'mean': arr.mean(), 'std': arr.std(), 'n': len(arr)}
        else:
            result[f] = {'mean': float('nan'), 'std': float('nan'), 'n': 0}
    return result


def _single_to_stats(rmse_dict: dict[str, float]) -> dict[str, dict]:
    """Wrap a single-run result in the same {filter: {mean, std, n}} format."""
    return {f: {'mean': v, 'std': 0.0, 'n': 1} for f, v in rmse_dict.items()}


def collect_data(multi_dir: Path | None) -> dict[str, dict[str, dict]]:
    """
    Collect RMSE statistics for all study conditions.
    Uses multi-run data where available, falls back to named single-run CSVs.

    Returns:
        {condition_key: {filter: {mean, std, n}}}
    """
    # Mapping: condition key → (multi_dir subdir name, fallback single CSV name)
    CONDITIONS = {
        'baseline':      ('baseline',      'filter_data_with_resampling.csv'),
        'qr_proc_low':   ('qr_proc_low',   'filter_data_sigma_process_low.csv'),
        'qr_proc_high':  ('qr_proc_high',  'filter_data_sigma_process_high.csv'),
        'qr_meas_low':   ('qr_meas_low',   'filter_data_sigma_meas_low.csv'),
        'qr_meas_high':  ('qr_meas_high',  'filter_data_sigma_meas_high.csv'),
    }

    data: dict[str, dict] = {}
    for cond_key, (subdir, fallback_name) in CONDITIONS.items():
        stats = {}

        # Try multi-run directory first
        if multi_dir is not None:
            cond_dir = multi_dir / subdir
            if cond_dir.is_dir():
                stats = load_multi_condition(cond_dir)
                if stats:
                    n = next(iter(stats.values()))['n']
                    print(f'  {cond_key}: multi-run ({n} runs)')

        # Fall back to named single-run CSV
        if not stats:
            fallback = LOG_DIR / fallback_name
            if fallback.exists():
                stats = _single_to_stats(load_single_csv(fallback))
                print(f'  {cond_key}: single-run fallback ({fallback_name})')
            else:
                print(f'  {cond_key}: [WARN] no data found — skipping')
                continue

        data[cond_key] = stats

    return data


# ── Plotting ──────────────────────────────────────────────────────────────────

def _draw_panel(
    ax: plt.Axes,
    conditions: list[tuple[str, str]],    # [(key, x-label), ...]
    data: dict,
    title: str,
    show_legend: bool = False,
    show_circularity_note: bool = False,
) -> None:
    """Draw one grouped-bar panel onto ax."""
    n_cond   = len(conditions)
    n_filt   = len(FILTERS)
    width    = 0.22                       # bar width
    offsets  = np.linspace(-(n_filt - 1) / 2, (n_filt - 1) / 2, n_filt) * width
    x_ticks  = np.arange(n_cond)

    for f_idx, f in enumerate(FILTERS):
        means, stds, has_error = [], [], []
        for cond_key, _ in conditions:
            if cond_key not in data:
                means.append(0.0); stds.append(0.0); has_error.append(False)
                continue
            s = data[cond_key][f]
            means.append(s['mean'])
            stds.append(s['std'] if s['n'] > 1 else 0.0)
            has_error.append(s['n'] > 1)

        x_pos = x_ticks + offsets[f_idx]
        bars  = ax.bar(
            x_pos, means,
            width=width,
            color=FILTER_COLORS[f],
            alpha=0.75,
            label=FILTER_LABELS[f],
            zorder=3,
        )
        # Error bars only where we have multiple runs
        for xi, (m, s, has_e) in enumerate(zip(x_pos, stds, has_error)):
            if has_e and s > 0:
                ax.errorbar(
                    xi, means[xi], yerr=s,
                    fmt='none', ecolor='#333333',
                    elinewidth=1.2, capsize=3, zorder=4,
                )
        # Value labels on bars
        for bar, m in zip(bars, means):
            if not np.isnan(m) and m > 0:
                ax.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 0.002,
                    f'{m:.3f}',
                    ha='center', va='bottom',
                    fontsize=6.5, color='#222222',
                    rotation=90 if m > 0.12 else 0,
                )

    ax.set_xticks(x_ticks)
    ax.set_xticklabels([lbl for _, lbl in conditions], fontsize=8.5)
    ax.set_title(title, fontweight='bold', pad=6)
    ax.set_ylabel('Final Cumulative RMSE [m]')
    ax.grid(True, axis='y', alpha=0.3, zorder=0)
    ax.set_ylim(bottom=0)

    if show_circularity_note:
        ax.annotate(
            '* KF/EKF collapse due to\n  /odom circularity',
            xy=(2, 0.012), xytext=(1.55, 0.06),
            fontsize=7, color='#555555',
            arrowprops=dict(arrowstyle='->', color='#888888', lw=0.8),
        )

    if show_legend:
        legend_handles = [
            Patch(facecolor=FILTER_COLORS[f], alpha=0.75, label=FILTER_LABELS[f])
            for f in FILTERS
        ]
        ax.legend(handles=legend_handles, loc='upper left')


def generate_figure(data: dict, out_path: Path) -> None:
    """Build and save the two-panel Q/R sensitivity figure."""
    fig, (ax_proc, ax_meas) = plt.subplots(
        1, 2, figsize=(12, 5), sharey=False,
    )
    fig.suptitle(
        'Q/R Sensitivity Study — Effect of Process and Measurement Noise on RMSE\n'
        'Error bars = ±1σ across 5 runs  |  n=1 shown without error bars',
        fontsize=10, fontweight='bold',
    )

    # ── Left: process noise ───────────────────────────────────────────────────
    proc_conditions = [
        ('baseline',     'Baseline\n(σ_p=0.01)'),
        ('qr_proc_low',  'Low\n(σ_p=0.001)'),
        ('qr_proc_high', 'High\n(σ_p=0.10)'),
    ]
    _draw_panel(
        ax_proc, proc_conditions, data,
        title='Process Noise (σ_p) — σ_m fixed at 0.10',
        show_legend=True,
    )

    # ── Right: measurement noise ───────────────────────────────────────────────
    meas_conditions = [
        ('baseline',     'Baseline\n(σ_m=0.10)'),
        ('qr_meas_low',  'Low *\n(σ_m=0.01)'),
        ('qr_meas_high', 'High\n(σ_m=0.50)'),
    ]
    _draw_panel(
        ax_meas, meas_conditions, data,
        title='Measurement Noise (σ_m) — σ_p fixed at 0.01',
        show_legend=False,
        show_circularity_note=True,
    )

    plt.tight_layout()
    plt.savefig(out_path, bbox_inches='tight')
    print(f'Saved → {out_path}')
    try:
        subprocess.Popen(['explorer.exe', str(out_path).replace('/', '\\')])
    except FileNotFoundError:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Resolve multi-run directory
    if len(sys.argv) >= 2:
        multi_dir = Path(sys.argv[1]).expanduser()
        if not multi_dir.is_dir():
            print(f'ERROR: {multi_dir} is not a directory.')
            sys.exit(1)
    else:
        candidates = sorted(LOG_DIR.glob('multi_*'), reverse=True)
        multi_dir  = candidates[0] if candidates else None
        if multi_dir:
            print(f'Auto-loading: {multi_dir}')
        else:
            print('No multi_* directory found — using single-run CSVs only.')

    print('\nLoading data...')
    data = collect_data(multi_dir)

    if not data:
        print('ERROR: No data found. Check ~/prob_ros_ws/logs/.')
        sys.exit(1)

    generate_figure(data, OUT_PATH)
    print('Done.')


if __name__ == '__main__':
    main()
