#!/usr/bin/env python3
"""
Tool to plot and compare planned vs actual drone paths.

Usage:
  python3 plot_paths.py                            # Auto-detect latest files, show all plots
  python3 plot_paths.py planned.csv actual.csv     # Specify files explicitly
  python3 plot_paths.py --2d                       # XY plane only
  python3 plot_paths.py --3d                       # 3D only
  python3 plot_paths.py --time                     # X/Y/Z vs time + error only
"""

import os
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import argparse


# ---------------------------------------------------------------------------
# File discovery
# ---------------------------------------------------------------------------

def find_latest_paths(home_dir=None):
    if home_dir is None:
        home_dir = os.path.expanduser('~')
    planned_files = glob.glob(os.path.join(home_dir, 'planned_path_*.csv'))
    actual_files  = glob.glob(os.path.join(home_dir, 'actual_path_*.csv'))
    if not planned_files or not actual_files:
        return None, None
    return max(planned_files, key=os.path.getctime), max(actual_files, key=os.path.getctime)


# ---------------------------------------------------------------------------
# Data loading + time axis
# ---------------------------------------------------------------------------

def load_path_csv(filepath):
    df = pd.read_csv(filepath)
    # Build absolute time [s] then normalise to start at 0
    if 'time_sec' in df.columns and 'time_nsec' in df.columns:
        df['t'] = df['time_sec'] + df['time_nsec'] * 1e-9
        df['t'] -= df['t'].iloc[0]
    else:
        # Fall back: treat row index as time
        df['t'] = df.index.astype(float)
    return df


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def plot_2d_xy(planned_df, actual_df):
    """Top-down XY comparison."""
    fig, ax = plt.subplots(figsize=(9, 8))
    if len(planned_df):
        ax.plot(planned_df['x'], planned_df['y'], 'b-', lw=1.8, label='Planned')
        ax.plot(planned_df['x'].iloc[0],  planned_df['y'].iloc[0],  'go', ms=10, label='Start')
        ax.plot(planned_df['x'].iloc[-1], planned_df['y'].iloc[-1], 'b^', ms=10, label='End (Planned)')
    if len(actual_df):
        ax.plot(actual_df['x'], actual_df['y'], 'r--', lw=1.8, label='Actual')
        ax.plot(actual_df['x'].iloc[-1], actual_df['y'].iloc[-1], 'rs', ms=10, label='End (Actual)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Path Comparison — XY Plane')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    plt.tight_layout()
    return fig


def plot_3d(planned_df, actual_df):
    """3-D trajectory comparison."""
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')
    if len(planned_df):
        ax.plot(planned_df['x'], planned_df['y'], planned_df['z'], 'b-', lw=1.5, label='Planned')
        ax.scatter(*planned_df[['x','y','z']].iloc[0],  c='g', s=80, marker='o')
        ax.scatter(*planned_df[['x','y','z']].iloc[-1], c='b', s=80, marker='^')
    if len(actual_df):
        ax.plot(actual_df['x'], actual_df['y'], actual_df['z'], 'r--', lw=1.5, label='Actual')
        ax.scatter(*actual_df[['x','y','z']].iloc[-1], c='r', s=80, marker='s')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Path Comparison — 3D')
    ax.legend()
    plt.tight_layout()
    return fig


def plot_time_domain(planned_df, actual_df):
    """
    6-subplot figure:
      Row 1-3: planned vs actual for x, y, z over time
      Row 4-6: tracking error for x, y, z over time (interpolated onto actual timeline)
    """
    t_p = planned_df['t'].values
    t_a = actual_df['t'].values

    axes_labels = ['x', 'y', 'z']
    colors_plan = ['#1f77b4', '#2ca02c', '#d62728']
    colors_act  = ['#ff7f0e', '#9467bd', '#8c564b']

    # Interpolate planned onto actual time grid for error computation
    err = {}
    for ax_name in axes_labels:
        p_interp = np.interp(t_a, t_p, planned_df[ax_name].values)
        err[ax_name] = actual_df[ax_name].values - p_interp

    fig, axes = plt.subplots(6, 1, figsize=(13, 16), sharex=False)
    fig.suptitle('Planned vs Actual — Time Domain', fontsize=14, y=1.001)

    # --- rows 0-2: signal comparison ---
    for i, ax_name in enumerate(axes_labels):
        ax = axes[i]
        ax.plot(t_p, planned_df[ax_name], color=colors_plan[i], lw=1.5, label='Planned')
        ax.plot(t_a, actual_df[ax_name],  color=colors_act[i],  lw=1.5, ls='--', label='Actual')
        ax.set_ylabel(f'{ax_name.upper()} (m)', fontsize=11)
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True, alpha=0.3)

    # --- rows 3-5: error ---
    for i, ax_name in enumerate(axes_labels):
        ax = axes[3 + i]
        ax.plot(t_a, err[ax_name], color='crimson', lw=1.2)
        ax.axhline(0, color='k', lw=0.8, ls='--')
        rmse = np.sqrt(np.mean(err[ax_name]**2))
        ax.set_ylabel(f'Error {ax_name.upper()} (m)\nRMSE={rmse:.3f}', fontsize=10)
        ax.grid(True, alpha=0.3)

    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[5].set_xlabel('Time (s)', fontsize=11)

    # Align x-limits
    t_max = max(t_p[-1], t_a[-1])
    for ax in axes:
        ax.set_xlim(left=0, right=t_max * 1.02)

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def print_statistics(planned_df, actual_df):
    def path_length(df):
        d = df[['x','y','z']].diff().fillna(0)
        return np.sqrt((d**2).sum(axis=1)).sum()

    t_a = actual_df['t'].values
    t_p = planned_df['t'].values

    print("\n" + "="*60)
    print("PATH STATISTICS")
    print("="*60)
    print(f"\nPlanned  — points: {len(planned_df)}, duration: {t_p[-1]:.1f}s, length: {path_length(planned_df):.2f}m")
    print(f"Actual   — points: {len(actual_df)},  duration: {t_a[-1]:.1f}s, length: {path_length(actual_df):.2f}m")

    for ax_name in ['x', 'y', 'z']:
        p_interp = np.interp(t_a, t_p, planned_df[ax_name].values)
        err = actual_df[ax_name].values - p_interp
        rmse = np.sqrt(np.mean(err**2))
        max_e = np.max(np.abs(err))
        print(f"  {ax_name.upper()} error — RMSE: {rmse:.4f} m   Max: {max_e:.4f} m")

    ep = planned_df[['x','y','z']].iloc[-1].values
    ea = actual_df[['x','y','z']].iloc[-1].values
    print(f"\nEndpoint error (3D): {np.linalg.norm(ea - ep):.4f} m")
    print("="*60 + "\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Compare planned vs actual drone paths.')
    parser.add_argument('planned_file', nargs='?', default=None)
    parser.add_argument('actual_file',  nargs='?', default=None)
    parser.add_argument('--2d',   dest='two_d',  action='store_true', help='XY plane plot only')
    parser.add_argument('--3d',   dest='three_d', action='store_true', help='3D plot only')
    parser.add_argument('--time', dest='time',   action='store_true', help='Time-domain plots only')
    parser.add_argument('--no-stats', dest='no_stats', action='store_true', help='Skip statistics')
    args = parser.parse_args()

    # Resolve files
    if args.planned_file is None or args.actual_file is None:
        print("Searching for latest path files...")
        planned_file, actual_file = find_latest_paths()
        if planned_file is None:
            print("ERROR: No planned_path_*.csv / actual_path_*.csv found in home directory.")
            sys.exit(1)
        print(f"Planned : {planned_file}")
        print(f"Actual  : {actual_file}")
    else:
        planned_file, actual_file = args.planned_file, args.actual_file

    try:
        planned_df = load_path_csv(planned_file)
        actual_df  = load_path_csv(actual_file)
    except Exception as e:
        print(f"ERROR loading CSV: {e}")
        sys.exit(1)

    if not args.no_stats:
        print_statistics(planned_df, actual_df)

    # Decide what to show (default: all three)
    show_2d   = args.two_d   or (not args.three_d and not args.time)
    show_3d   = args.three_d or (not args.two_d   and not args.time)
    show_time = args.time    or (not args.two_d   and not args.three_d)

    if show_2d:
        plot_2d_xy(planned_df, actual_df)
    if show_3d:
        plot_3d(planned_df, actual_df)
    if show_time:
        plot_time_domain(planned_df, actual_df)

    plt.show()


if __name__ == '__main__':
    main()

