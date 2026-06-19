#!/usr/bin/env python3
"""
Plot Results — Probabilistic Robot Lab
=======================================
Reads CSV from data_logger.py and generates paper-ready figures:

  Panel 1 — 2D Trajectory        odom vs KF vs EKF vs PF
  Panel 2 — Position Error        ||p_filter - p_odom|| over time
  Panel 3 — Cumulative RMSE       KF vs EKF vs PF (final value in legend)
  Panel 4 — Covariance Trace      trace(Σ) = Σ_xx + Σ_yy + Σ_θθ
  Panel 5 — N_eff (if PF present) effective sample size (Special Task)

Usage:
    python3 plot_results.py                     # auto-loads latest log
    python3 plot_results.py path/to/log.csv

Output:
    <basename>_plots.pdf

Dependencies:
    pip install matplotlib pandas --break-system-packages
"""

import sys
import os
import subprocess
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

matplotlib.rcParams.update({
    'font.size': 9, 'axes.titlesize': 10,
    'axes.labelsize': 9, 'legend.fontsize': 8,
    'figure.dpi': 150,
})

LOG_DIR = os.path.expanduser('~/prob_ros_ws/logs')

# ---------------------------------------------------------------------------
# Waypoints — must match WAYPOINTS in auto_nav.py  [x, y, theta_deg]
# ---------------------------------------------------------------------------
# todo: DRY this up by importing from auto_nav.py or a shared config file
WAYPOINTS = [
    (3.8,  1.0, 180.0),   # WP1 — north
    (2.0,  2.8, 270.0),   # WP2 — west
    (0.2,  1.0,   0.0),   # WP3 — south
    (2.0, -0.8,  90.0),   # WP4 — east
    (-1.8,  0.0, 270.0),   # west
    ( 0.0,  1.8,  180.0),   # north
    ( 0.0, -1.8, 0.0),   # south
    (-1.8,  0.0, 270.0),   # west
]


def pos_error(df, prefix):
    dx = df[f'{prefix}_x'] - df['odom_x']
    dy = df[f'{prefix}_y'] - df['odom_y']
    return np.sqrt(dx**2 + dy**2)


def cumulative_rmse(err):
    sq = err.fillna(0.0) ** 2
    return np.sqrt(np.cumsum(sq) / (np.arange(len(sq)) + 1))


def cov_trace(df, prefix):
    return df[f'{prefix}_cov_xx'] + df[f'{prefix}_cov_yy'] + df[f'{prefix}_cov_tt']


def has_data(df, prefix):
    return f'{prefix}_x' in df.columns and df[f'{prefix}_x'].notna().any()


def plot(df, out_path):
    t      = df['time_s']
    colors = {'kf': '#1f77b4', 'ekf': '#d62728', 'pf': '#2ca02c'}
    labels = {'kf': 'KF',      'ekf': 'EKF',     'pf': 'PF'}
    active = [p for p in ('kf', 'ekf', 'pf') if has_data(df, p)]
    has_neff = 'n_eff' in df.columns and df['n_eff'].notna().any()

    ncols = 2
    nrows = 3 if (has_neff and 'pf' in active) else 2
    fig = plt.figure(figsize=(13, 5 * nrows))
    fig.suptitle(
        'Probabilistic Robot Lab — KF / EKF / PF Filter Comparison\n'
        'Ground truth: /odom (Gazebo)',
        fontsize=11, fontweight='bold', y=0.99)

    gs = gridspec.GridSpec(nrows, ncols, figure=fig, hspace=0.42, wspace=0.32)

    # ---- Panel 1: 2D Trajectory ----
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df['odom_x'], df['odom_y'], 'k-', lw=2, label='Odom (GT)', zorder=10)
    for p in active:
        ax1.plot(df[f'{p}_x'], df[f'{p}_y'], '--',
                 color=colors[p], lw=1.3, label=labels[p], alpha=0.85)
    ax1.plot(df['odom_x'].iloc[0],  df['odom_y'].iloc[0],  'ko', ms=6, zorder=11, label='Start')
    ax1.plot(df['odom_x'].iloc[-1], df['odom_y'].iloc[-1], 'k^', ms=6, zorder=11, label='End')
    # Waypoints — gold stars numbered WP1…WPn
    for i, (wx, wy, _) in enumerate(WAYPOINTS):
        ax1.plot(wx, wy, marker='*', color='#DAA520', ms=12, zorder=12,
                 label='Goals' if i == 0 else None)
        ax1.annotate(f'WP{i + 1}', xy=(wx, wy),
                     xytext=(4, 6), textcoords='offset points',
                     fontsize=7, color='#8B6914', fontweight='bold')
    ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]')
    ax1.set_title('2D Trajectory'); ax1.legend(); ax1.set_aspect('equal', 'datalim')
    ax1.grid(True, alpha=0.3)

    # ---- Panel 2: Position Error ----
    ax2 = fig.add_subplot(gs[0, 1])
    for p in active:
        ax2.plot(t, pos_error(df, p), color=colors[p], lw=1.2, label=labels[p])
    ax2.set_xlabel('Time [s]'); ax2.set_ylabel('‖p_filter − p_odom‖ [m]')
    ax2.set_title('Position Error over Time'); ax2.legend(); ax2.grid(True, alpha=0.3)

    # ---- Panel 3: Cumulative RMSE ----
    ax3 = fig.add_subplot(gs[1, 0])
    for p in active:
        rmse  = cumulative_rmse(pos_error(df, p))
        ax3.plot(t, rmse, color=colors[p], lw=1.5,
                 label=f'{labels[p]}  RMSE={rmse.iloc[-1]:.3f} m')
    ax3.set_xlabel('Time [s]'); ax3.set_ylabel('Cumulative RMSE [m]')
    ax3.set_title('Cumulative RMSE vs Odometry'); ax3.legend(); ax3.grid(True, alpha=0.3)

    # ---- Panel 4: Covariance Trace ----
    ax4 = fig.add_subplot(gs[1, 1])
    for p in active:
        ax4.plot(t, cov_trace(df, p), color=colors[p], lw=1.2, label=labels[p])
    ax4.set_xlabel('Time [s]'); ax4.set_ylabel('trace(Σ) = Σ_xx + Σ_yy + Σ_θθ')
    ax4.set_title('Covariance Trace (Total Uncertainty)'); ax4.legend(); ax4.grid(True, alpha=0.3)

    # ---- Panel 5: N_eff (PF only) ----
    if nrows == 3:
        ax5 = fig.add_subplot(gs[2, :])
        ax5.plot(t, df['n_eff'], color=colors['pf'], lw=1.4, label='N_eff')
        ax5.set_xlabel('Time [s]')
        ax5.set_ylabel('N_eff = 1/Σwᵢ²')
        ax5.set_title('Effective Sample Size — Particle Diversity (Special Task)')
        ax5.legend(); ax5.grid(True, alpha=0.3)

    plt.savefig(out_path, bbox_inches='tight')
    print(f'Saved: {out_path}')
    subprocess.Popen(['explorer.exe', out_path.replace('/', '\\')])


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
    else:
        try:
            files = sorted(f for f in os.listdir(LOG_DIR)
                           if f.startswith('filter_data') and f.endswith('.csv'))
            csv_path = os.path.join(LOG_DIR, files[-1])
            print(f'Auto-loading: {csv_path}')
        except (FileNotFoundError, IndexError):
            print('No log files found in ~/prob_ros_ws/logs/')
            print('Usage: python3 plot_results.py [path_to_csv]')
            sys.exit(1)

    out_path = os.path.splitext(csv_path)[0] + '_plots.pdf'
    df = pd.read_csv(csv_path)
    plot(df, out_path)