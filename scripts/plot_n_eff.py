#!/usr/bin/env python3
"""
Plot N_eff — Special Task Particle Degeneration Analysis
=========================================================
Compares N_eff (Effective Sample Size) between:
  - PF with resampling    (healthy, N_eff ≈ N)
  - PF without resampling (degenerates, N_eff → 1)

Usage:
    # Compare two CSV files
    python3 plot_n_eff.py \\
        ~/prob_ros_ws/logs/filter_data_WITH_resampling.csv \\
        ~/prob_ros_ws/logs/filter_data_NO_resampling.csv

    # Single file (shows only what's available)
    python3 plot_n_eff.py ~/prob_ros_ws/logs/filter_data_20260619_120000.csv

Output:
    <csv_basename>_n_eff.pdf

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

matplotlib.rcParams.update({'font.size': 9, 'figure.dpi': 150})

LOG_DIR = os.path.expanduser('~/prob_ros_ws/logs')


def latest_csv(exclude=None):
    files = sorted(
        f for f in os.listdir(LOG_DIR)
        if f.startswith('filter_data') and f.endswith('.csv') and f != exclude
    )
    return os.path.join(LOG_DIR, files[-1]) if files else None


def load(path):
    df = pd.read_csv(path)
    # Normalise N_eff if num_particles column is missing (divide by row count max)
    if 'n_eff' not in df.columns:
        raise ValueError(f"No 'n_eff' column in {path}")
    return df


def pos_error(df, prefix):
    dx = df[f'{prefix}_x'] - df['odom_x']
    dy = df[f'{prefix}_y'] - df['odom_y']
    return np.sqrt(dx**2 + dy**2)


def cumulative_rmse(err):
    sq = err.fillna(0.0) ** 2
    return np.sqrt(np.cumsum(sq) / (np.arange(len(sq)) + 1))


def plot_single(df, label, ax_neff, ax_rmse, color, n_particles=500):
    t = df['time_s']

    # N_eff normalised to [0,1]
    n_eff_norm = df['n_eff'] / n_particles
    ax_neff.plot(t, n_eff_norm, color=color, lw=1.4, label=label)

    # RMSE if PF data available
    if df['pf_x'].notna().any():
        err  = pos_error(df, 'pf')
        rmse = cumulative_rmse(err)
        ax_rmse.plot(t, rmse, color=color, lw=1.4,
                     label=f'{label}  (RMSE={rmse.iloc[-1]:.3f} m)')


def main():
    paths = []

    if len(sys.argv) >= 2:
        paths.append(sys.argv[1])
    else:
        p = latest_csv()
        if p:
            paths.append(p)
            print(f'Auto-loading: {p}')

    if len(sys.argv) >= 3:
        paths.append(sys.argv[2])

    if not paths:
        print('No CSV files found.')
        print('Usage: python3 plot_n_eff.py [with_resampling.csv] [no_resampling.csv]')
        sys.exit(1)

    colors = ['#1f77b4', '#d62728']
    labels = ['PF (resampling ON)', 'PF (resampling OFF — Special Task)']

    fig = plt.figure(figsize=(13, 7))
    fig.suptitle(
        'Special Task 2510331009 — Particle Degeneration Analysis\n'
        'N_eff = 1/Σ wᵢ²   ∈ [1/N, 1]   (normalised to [0, 1])',
        fontsize=11, fontweight='bold')

    gs = gridspec.GridSpec(2, 1, figure=fig, hspace=0.38)
    ax_neff = fig.add_subplot(gs[0])
    ax_rmse = fig.add_subplot(gs[1])

    for i, path in enumerate(paths):
        try:
            df = load(path)
            lbl = labels[i] if i < len(labels) else os.path.basename(path)
            plot_single(df, lbl, ax_neff, ax_rmse,
                        colors[i % len(colors)])
        except Exception as e:
            print(f'Error loading {path}: {e}')

    # Reference line: full degeneration
    ax_neff.axhline(y=1.0/500, color='gray', ls=':', lw=1, label='N_eff = 1 (full degeneration)')
    ax_neff.axhline(y=1.0,     color='gray', ls='--', lw=1, label='N_eff = N (ideal diversity)')
    ax_neff.set_xlabel('Time [s]')
    ax_neff.set_ylabel('N_eff / N')
    ax_neff.set_title('Normalised Effective Sample Size over Time')
    ax_neff.legend(fontsize=8)
    ax_neff.set_ylim(-0.05, 1.1)
    ax_neff.grid(True, alpha=0.3)

    ax_rmse.set_xlabel('Time [s]')
    ax_rmse.set_ylabel('Cumulative RMSE [m]')
    ax_rmse.set_title('PF Position RMSE vs Odometry Ground Truth')
    ax_rmse.legend(fontsize=8)
    ax_rmse.grid(True, alpha=0.3)

    out_path = os.path.splitext(paths[0])[0] + '_n_eff.pdf'
    plt.savefig(out_path, bbox_inches='tight')
    print(f'Saved: {out_path}')
    subprocess.Popen(['xdg-open', out_path])


if __name__ == '__main__':
    main()
