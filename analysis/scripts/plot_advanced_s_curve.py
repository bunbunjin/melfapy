#!/usr/bin/env python3
import sys
from pathlib import Path
import argparse
import csv

# make src importable
# avoid static-type warning from some linters by inserting without using insert(index,...)
_src_path = str(Path(__file__).parent.parent.parent / "src")
sys.path[0:0] = [_src_path]
from melfapy.utils.advanced_S_curve_acceleration import AdvancedSCurvePlanner

import numpy as np
import matplotlib.pyplot as plt

# Increase default font sizes for better readability in saved PNGs
plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 16,
    'axes.labelsize': 14,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12,
})


RESULTS_DIR = Path(__file__).parent.parent / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)


def save_profile_csv(path: Path, t_array, pos, vel, acc, jerk):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "pos", "vel", "acc", "jerk"])
        # use zip to avoid range(len(...)) and related static type warnings
        for t, p, v, a, j in zip(t_array, pos, vel, acc, jerk):
            writer.writerow([f"{t:.6f}", f"{p:.12f}", f"{v:.12f}", f"{a:.12f}", f"{j:.12f}"])


def plot_and_save(t_array, pos, vel, acc, jerk, planner, name: str):
    """
    各信号ごとに個別の PNG を作成して保存する。
    """
    png_paths = {}

    # position
    fig_pos, ax_pos = plt.subplots(figsize=(10, 3))
    ax_pos.plot(t_array, pos, "b")
    ax_pos.set_ylabel("position (m)")
    ax_pos.set_title(f"Position ({name})")
    ax_pos.set_xlabel("time (s)")
    # ensure x-limits cover full motion and ticks are visible
    if getattr(planner, 'T', None) is not None:
        ax_pos.set_xlim(0, planner.T)
    ax_pos.tick_params(axis='x', which='both', labelrotation=0)
    ax_pos.grid(True)
    png_pos = RESULTS_DIR / f"advanced_s_curve_{name}_pos.png"
    fig_pos.tight_layout()
    fig_pos.savefig(png_pos, bbox_inches='tight')
    plt.close(fig_pos)
    png_paths['pos'] = png_pos

    # velocity
    fig_vel, ax_vel = plt.subplots(figsize=(10, 3))
    ax_vel.plot(t_array, vel, "g")
    ax_vel.set_ylabel("velocity (m/s)")
    ax_vel.set_title(f"Velocity ({name})")
    ax_vel.set_xlabel("time (s)")
    if getattr(planner, 'T', None) is not None:
        ax_vel.set_xlim(0, planner.T)
    ax_vel.axhline(y=planner.v_lim * planner.sign, color="r", linestyle="--")
    ax_vel.axhline(y=-planner.v_lim * planner.sign, color="r", linestyle="--")
    ax_vel.tick_params(axis='x', which='both', labelrotation=0)
    ax_vel.grid(True)
    png_vel = RESULTS_DIR / f"advanced_s_curve_{name}_vel.png"
    fig_vel.tight_layout()
    fig_vel.savefig(png_vel, bbox_inches='tight')
    plt.close(fig_vel)
    png_paths['vel'] = png_vel

    # acceleration
    fig_acc, ax_acc = plt.subplots(figsize=(10, 3))
    ax_acc.plot(t_array, acc, "r")
    ax_acc.set_ylabel("acceleration (m/s^2)")
    ax_acc.set_title(f"Acceleration ({name})")
    ax_acc.set_xlabel("time (s)")
    if getattr(planner, 'T', None) is not None:
        ax_acc.set_xlim(0, planner.T)
    ax_acc.axhline(y=planner.a_lim * planner.sign, color="r", linestyle="--")
    ax_acc.axhline(y=-planner.a_lim * planner.sign, color="r", linestyle="--")
    ax_acc.tick_params(axis='x', which='both', labelrotation=0)
    ax_acc.grid(True)
    png_acc = RESULTS_DIR / f"advanced_s_curve_{name}_acc.png"
    fig_acc.tight_layout()
    fig_acc.savefig(png_acc, bbox_inches='tight')
    plt.close(fig_acc)
    png_paths['acc'] = png_acc

    # jerk
    fig_j, ax_j = plt.subplots(figsize=(10, 3))
    ax_j.plot(t_array, jerk, color="purple")
    ax_j.set_ylabel("jerk (m/s^3)")
    ax_j.set_title(f"Jerk ({name})")
    ax_j.set_xlabel("time (s)")
    if getattr(planner, 'T', None) is not None:
        ax_j.set_xlim(0, planner.T)
    ax_j.tick_params(axis='x', which='both', labelrotation=0)
    ax_j.grid(True)
    png_j = RESULTS_DIR / f"advanced_s_curve_{name}_jerk.png"
    fig_j.tight_layout()
    fig_j.savefig(png_j, bbox_inches='tight')
    plt.close(fig_j)
    png_paths['jerk'] = png_j

    return png_paths


def compute_profile(planner, samples=1000):
    if planner.T == 0:
        return np.array([0.0]), np.array([planner.q0]), np.array([0.0]), np.array([0.0]), np.array([0.0])
    t_array = np.linspace(0.0, planner.T, samples)
    pos = np.empty_like(t_array)
    vel = np.empty_like(t_array)
    acc = np.empty_like(t_array)
    jerk = np.empty_like(t_array)
    for i, t in enumerate(t_array):
        p, v, a, j = planner.get_profile(t)
        pos[i], vel[i], acc[i], jerk[i] = p, v, a, j
    return t_array, pos, vel, acc, jerk


def main():
    parser = argparse.ArgumentParser(description="Plot Advanced S-curve profiles and save CSV/PNG outputs.")
    parser.add_argument("--q0", type=float, default=0.0)
    parser.add_argument("--q1", type=float, default=1.0)
    parser.add_argument("--v_max", type=float, default=1.0)
    parser.add_argument("--a_max", type=float, default=2.0)
    parser.add_argument("--j_max", type=float, default=10.0)
    parser.add_argument("--name", type=str, default="default")
    parser.add_argument("--samples", type=int, default=1000)
    args = parser.parse_args()

    planner = AdvancedSCurvePlanner(args.q0, args.q1, args.v_max, args.a_max, args.j_max)

    t_array, pos, vel, acc, jerk = compute_profile(planner, samples=args.samples)

    csv_path = RESULTS_DIR / f"advanced_s_curve_{args.name}.csv"
    save_profile_csv(csv_path, t_array, pos, vel, acc, jerk)

    png_paths = plot_and_save(t_array, pos, vel, acc, jerk, planner, args.name)

    print(f"Saved CSV: {csv_path}")
    for key, path in png_paths.items():
        print(f"Saved PNG ({key}): {path}")


if __name__ == "__main__":
    main()