#!/usr/bin/env python3
"""
Plot Results — Probabilistic Robot Lab
=======================================
Reads the CSV from data_logger.py and generates a 4-panel figure for the paper:

  Panel 1 — 2D Trajectory       : Odom (ground truth) vs KF vs EKF vs PF
  Panel 2 — Position Error      : ||p_filter − p_odom|| over time [m]
  Panel 3 — Cumulative RMSE     : KF vs EKF vs PF, final value in legend
  Panel 4 — Covariance Trace    : trace(Σ) = Σ_xx + Σ_yy + Σ_θθ over time

Usage:
    # Auto-loads the latest log in ~/prob_ros_ws/logs/
    python3 plot_results.py

    # Explicit file
    python3 plot_results.py ~/prob_ros_ws/logs/filter_data_20260618_120000.csv

Output:
    <csv_basename>_plots.pdf   (same directory as the CSV)

Dependencies:
    pip install matplotlib pandas --break-system-packages
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

matplotlib.rcParams.update({
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'figure.dpi': 150,
})

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pos_error(df: pd.DataFrame, prefix: str) -> pd.Series:
    """Euclidean distance between filter estimate and odom (ground truth)."""
    dx = df[f'{prefix}_x'] - df['odom_x']
    dy = df[f'{prefix}_y'] - df['odom_y']
    return np.sqrt(dx**2 + dy**2)


def cumulative_rmse(err_series: pd.Series) -> np.ndarray:
    """Cumulative RMSE: sqrt( mean of squared errors up to each time step )."""
    sq = err_series.fillna(0.0) ** 2
    return np.sqrt(np.cumsum(sq) / (np.arange(len(sq)) + 1))


def cov_trace(df: pd.DataFrame, prefix: str) -> pd.Series:
    """trace(Σ) = Σ_xx + Σ_yy + Σ_θθ  — scalar measure of total uncertainty."""
    return df[f'{prefix}_cov_xx'] + df[f'{prefix}_cov_yy'] + df[f'{prefix}_cov_tt']


def has_data(df: pd.DataFrame, prefix: str) -> bool:
    return df[f'{prefix}_x'].notna().any()


# ---------------------------------------------------------------------------
# Main plot
# ---------------------------------------------------------------------------

def plot(df: pd.DataFrame, out_path: str) -> None:
    t = df['time_s']

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle(
        'Probabilistic Robot Lab — KF / EKF / PF Filter Comparison\n'
        'Ground truth: /odom (Gazebo odometry)',
        fontsize=11, fontweight='bold', y=0.98)

    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.40, wspace=0.32)

    colors = {'kf': '#1f77b4', 'ekf': '#d62728', 'pf': '#2ca02c'}
    labels = {'kf': 'KF', 'ekf': 'EKF', 'pf': 'PF'}

    active = [p for p in ('kf', 'ekf', 'pf') if has_data(df, p)]

    # -------------------------------------------------------------------
    # Panel 1 — 2D Trajectory
    # -------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df['odom_x'], df['odom_y'],
             'k-', lw=2.0, label='Odom (GT)', zorder=10)
    for p in active:
        ax1.plot(df[f'{p}_x'], df[f'{p}_y'],
                 '--', color=colors[p], lw=1.3, label=labels[p], alpha=0.85)

    # Mark start and end
    ax1.plot(df['odom_x'].iloc[0],  df['odom_y'].iloc[0],
             'ko', ms=6, zorder=11, label='Start')
    ax1.plot(df['odom_x'].iloc[-1], df['odom_y'].iloc[-1],
             'k^', ms=6, zorder=11, label='End')

    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('y [m]')
    ax1.set_title('2D Trajectory')
    ax1.legend()
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.grid(True, alpha=0.3)

    # -------------------------------------------------------------------
    # Panel 2 — Position error over time
    # -------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    for p in active:
        err = pos_error(df, p)
        ax2.plot(t, err, '-', color=colors[p], lw=1.2, label=labels[p])

    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('‖p_filter − p_odom‖ [m]')
    ax2.set_title('Position Error over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # -------------------------------------------------------------------
    # Panel 3 — Cumulative RMSE
    # -------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 0])
    for p in active:
        err   = pos_error(df, p)
        rmse  = cumulative_rmse(err)
        ax3.plot(t, rmse, '-', color=colors[p], lw=1.5,
                 label=f'{labels[p]}  (RMSE = {rmse.iloc[-1]:.3f} m)')

    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Cumulative RMSE [m]')
    ax3.set_title('Cumulative RMSE vs Odometry Ground Truth')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # -------------------------------------------------------------------
    # Panel 4 — Covariance trace
    # -------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 1])
    for p in active:
        tr = cov_trace(df, p)
        ax4.plot(t, tr, '-', color=colors[p], lw=1.2, label=labels[p])

    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('trace(Σ) = Σ_xx + Σ_yy + Σ_θθ')
    ax4.set_title('Covariance Trace (Total Uncertainty)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # -------------------------------------------------------------------
    plt.savefig(out_path, bbox_inches='tight')
    print(f'Saved: {out_path}')
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    log_dir = os.path.expanduser('~/prob_ros_ws/logs')

    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
    else:
        # Auto-load the most recent log file
        try:
            files = sorted(
                f for f in os.listdir(log_dir)
                if f.startswith('filter_data') and f.endswith('.csv')
            )
            if not files:
                raise FileNotFoundError
            csv_path = os.path.join(log_dir, files[-1])
            print(f'Auto-loading: {csv_path}')
        except FileNotFoundError:
            print('No log files found in ~/prob_ros_ws/logs/')
            print('Usage: python3 plot_results.py [path_to_csv]')
            sys.exit(1)

    out_path = os.path.splitext(csv_path)[0] + '_plots.pdf'
    df = pd.read_csv(csv_path)
    plot(df, out_path)
