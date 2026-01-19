#!/usr/bin/env python3
"""
Plot 3 trajectories in 3D from TUM format files:
timestamp tx ty tz qx qy qz qw

Files expected (default):
- ground_truth.txt
- odometry_ekf.txt
- odometry_orb.txt
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_tum_positions(path: str) -> np.ndarray:
    """
    Load positions (tx, ty, tz) from a TUM trajectory file.
    Ignores blank lines and comment lines starting with '#'.
    Returns Nx3 array.
    """
    positions = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                # Not enough columns to be TUM pose format
                continue
            # parts: [t, tx, ty, tz, qx, qy, qz, qw, ...]
            try:
                tx, ty, tz = map(float, parts[1:4])
                positions.append([tx, ty, tz])
            except ValueError:
                # Skip malformed lines
                continue

    if not positions:
        raise ValueError(f"No valid pose lines found in {path}")
    return np.asarray(positions, dtype=float)


def set_axes_equal(ax):
    """
    Make 3D plot axes have equal scale so trajectories don't look distorted.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def plot_traj(ax, P: np.ndarray, label: str, linewidth: float = 2.0, alpha: float = 1.0):
    ax.plot(P[:, 0], P[:, 1], P[:, 2], label=label, linewidth=linewidth, alpha=alpha)
    # Mark start + end
    ax.scatter(P[0, 0], P[0, 1], P[0, 2], marker="o", s=40)
    ax.scatter(P[-1, 0], P[-1, 1], P[-1, 2], marker="x", s=60)


def main():
    parser = argparse.ArgumentParser(description="3D plot of TUM-format trajectories.")
    parser.add_argument("--gt", default="ground_truth.txt", help="Ground truth TUM file")
    parser.add_argument("--ekf", default="odometry_ekf.txt", help="EKF odometry TUM file")
    parser.add_argument("--orb", default="odometry_orb.txt", help="ORB odometry TUM file")
    parser.add_argument("--title", default="3D Trajectories", help="Plot title")
    args = parser.parse_args()

    gt = load_tum_positions(args.gt)
    ekf = load_tum_positions(args.ekf)
    # orb = load_tum_positions(args.orb)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    plot_traj(ax, gt, "Ground Truth", linewidth=2.5, alpha=0.95)
    plot_traj(ax, ekf, "Odometry EKF", linewidth=2.0, alpha=0.9)
    # plot_traj(ax, orb, "Odometry ORB", linewidth=2.0, alpha=0.9)

    ax.set_title(args.title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend(loc="best")

    # Improve viewing
    set_axes_equal(ax)
    ax.view_init(elev=20, azim=-60)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
