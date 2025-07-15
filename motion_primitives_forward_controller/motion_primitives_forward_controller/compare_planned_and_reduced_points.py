#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Mathias Fuhrer

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def plot_cartesian_trajectory(filepath_planned, filepath_reduced, pose_names):
    # Unpack column names from pose_names list
    px, py, pz, qx, qy, qz, qw = pose_names

    # Load data from CSV files
    df_planned = pd.read_csv(filepath_planned)
    df_reduced = pd.read_csv(filepath_reduced)

    # Extract position coordinates
    x, y, z = df_planned[px], df_planned[py], df_planned[pz]
    xr, yr, zr = df_reduced[px], df_reduced[py], df_reduced[pz]

    # Prepare 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot planned and reduced paths
    ax.plot(x, y, z, marker="o", markersize=5, label="Planned Path", color="blue", alpha=0.5)
    ax.plot(xr, yr, zr, marker="o", markersize=5, label="Reduced Path", color="orange")

    # Mark start and end of the planned path
    ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color="red", s=50, label="Start")
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color="green", s=50, label="End")

    arrow_len = 0.05  # Length of coordinate axis arrows

    # Draw coordinate systems for all reduced points
    for _, row in df_reduced.iterrows():
        quat = [row[qx], row[qy], row[qz], row[qw]]
        rot = R.from_quat(quat)
        pt = [row[px], row[py], row[pz]]

        ax.quiver(*pt, *rot.apply([1, 0, 0]), length=arrow_len, color="r", normalize=True)
        ax.quiver(*pt, *rot.apply([0, 1, 0]), length=arrow_len, color="g", normalize=True)
        ax.quiver(*pt, *rot.apply([0, 0, 1]), length=arrow_len, color="b", normalize=True)

    # Draw coordinate system at the first point of the planned path,
    # using the orientation from the first reduced point
    first_quat = [
        df_reduced.iloc[0][qx],
        df_reduced.iloc[0][qy],
        df_reduced.iloc[0][qz],
        df_reduced.iloc[0][qw],
    ]
    first_rot = R.from_quat(first_quat)
    first_pt = [x.iloc[0], y.iloc[0], z.iloc[0]]

    ax.quiver(
        *first_pt,
        *first_rot.apply([1, 0, 0]),
        length=arrow_len,
        color="r",
        linestyle="dashed",
        normalize=True,
    )
    ax.quiver(
        *first_pt,
        *first_rot.apply([0, 1, 0]),
        length=arrow_len,
        color="g",
        linestyle="dashed",
        normalize=True,
    )
    ax.quiver(
        *first_pt,
        *first_rot.apply([0, 0, 1]),
        length=arrow_len,
        color="b",
        linestyle="dashed",
        normalize=True,
    )

    # Axis labels and title
    ax.set_xlabel("X in m")
    ax.set_ylabel("Y in m")
    ax.set_zlabel("Z in m")
    ax.set_title("Cartesian Trajectory: Planned vs. Reduced")
    ax.legend()

    # Set equal aspect ratio for all axes
    ranges = np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()])
    max_range = ranges.max() / 2.0
    mid = [x.mean(), y.mean(), z.mean()]
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    plt.tight_layout()

    # Save figure
    base_name = os.path.basename(filepath_planned).replace(
        "_planned.csv", "_compare_planned_vs_reduced_LIN_cartesian.png"
    )
    plot_path = os.path.join(os.path.dirname(filepath_planned), base_name)
    plt.savefig(plot_path)
    plt.show()
    print(f"Figure with planned and reduced points comparison saved to: {plot_path}")


def plot_joint_trajectory(filepath_planned, filepath_reduced, joint_names):
    df_planned = pd.read_csv(filepath_planned)
    df_reduced = pd.read_csv(filepath_reduced)

    joint_columns_planned = [name for name in joint_names]
    joint_columns_reduced = df_reduced.columns.tolist()

    planned = df_planned[joint_columns_planned].to_numpy()
    reduced = df_reduced[joint_columns_reduced].to_numpy()

    # Insert the first planned point at the beginning of reduced
    reduced = np.vstack([planned[0], reduced])

    # Find indices in planned trajectory where reduced points occur
    reduced_indices = []
    for red_point in reduced:
        for idx, plan_point in enumerate(planned):
            if np.allclose(red_point, plan_point, atol=1e-6):  # match with tolerance
                reduced_indices.append(idx)
                break
        else:
            reduced_indices.append(None)  # no match found

    # Prepare subplots
    fig, axs = plt.subplots(len(joint_names), 1, figsize=(10, 2.5 * len(joint_names)), sharex=True)

    if len(joint_names) == 1:
        axs = [axs]

    for i, joint in enumerate(joint_names):
        axs[i].plot(
            planned[:, i], marker="o", markersize=5, label="Planned", color="blue", alpha=0.5
        )

        # Filter valid matches for plotting
        valid_reduced = [
            (idx, reduced[j, i]) for j, idx in enumerate(reduced_indices) if idx is not None
        ]
        if valid_reduced:
            x_vals, y_vals = zip(*valid_reduced)
            axs[i].plot(x_vals, y_vals, marker="o", markersize=5, label="Reduced", color="orange")

        axs[i].set_ylabel("Angle in radians")
        axs[i].set_title(joint)
        axs[i].set_ylim(-3.2, 3.2)
        axs[i].grid(True)

    axs[-1].set_xlabel("Trajectory Point Index")
    # fig.suptitle("Joint Trajectory: Planned vs. Reduced", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.legend(
        ["Planned", "Reduced"],
        loc="lower right",
        bbox_to_anchor=(1, 0),
        bbox_transform=fig.transFigure,
        ncol=2,
    )

    # Save figure
    base_name = os.path.basename(filepath_planned).replace(
        "_planned.csv", "_compare_planned_vs_reduced_PTP_joint.png"
    )
    plot_path = os.path.join(os.path.dirname(filepath_planned), base_name)
    plt.savefig(plot_path)
    plt.show()
    print(f"Figure with planned and reduced points comparison saved to: {plot_path}")


def main():
    data_dir = "src/motion_primitives_forward_controller/data"

    filename_planned = "trajectory_<date>_planned.csv"
    filename_reduced = "trajectory_<date>_reduced_LIN_cartesian.csv"
    mode = "cartesian"

    # filename_planned = "trajectory_<date>_planned.csv"
    # filename_reduced = "trajectory_<date>_reduced_PTP_joint.csv"
    # mode = "joint"

    filepath_planned = os.path.join(data_dir, filename_planned)
    filepath_reduced = os.path.join(data_dir, filename_reduced)

    joint_names = [
        "shoulder_pan_joint_pos",
        "shoulder_lift_joint_pos",
        "elbow_joint_pos",
        "wrist_1_joint_pos",
        "wrist_2_joint_pos",
        "wrist_3_joint_pos",
    ]
    pose_names = ["pose_x", "pose_y", "pose_z", "pose_qx", "pose_qy", "pose_qz", "pose_qw"]

    if mode == "cartesian":
        plot_cartesian_trajectory(filepath_planned, filepath_reduced, pose_names)
    elif mode == "joint":
        plot_joint_trajectory(filepath_planned, filepath_reduced, joint_names)


if __name__ == "__main__":
    main()
