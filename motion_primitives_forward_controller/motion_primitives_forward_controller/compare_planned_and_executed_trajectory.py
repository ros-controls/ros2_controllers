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

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os


def compare_and_plot_joint_trajectories(
    filepath_planned, filepath_executed, joint_pos_names, n_points
):
    # Load CSV files
    df_planned = pd.read_csv(filepath_planned)
    df_executed = pd.read_csv(filepath_executed)

    # Remove leading/trailing rows of executed trajectory where all velocities are zero
    vel_cols = [col for col in df_executed.columns if "vel" in col]
    moving_mask = ~(df_executed[vel_cols] == 0).all(axis=1)
    start_index = moving_mask.idxmax()
    end_index = moving_mask[::-1].idxmax()
    df_executed_clean = df_executed.loc[start_index:end_index].reset_index(drop=True)

    # Resample planned trajectory
    planned_positions = df_planned[joint_pos_names].values
    interp_planned = interp1d(np.linspace(0, 1, len(planned_positions)), planned_positions, axis=0)
    planned_resampled = interp_planned(np.linspace(0, 1, n_points))

    # Resample executed trajectory
    executed_positions = df_executed_clean[joint_pos_names].values
    interp_executed = interp1d(
        np.linspace(0, 1, len(executed_positions)), executed_positions, axis=0
    )
    executed_resampled = interp_executed(np.linspace(0, 1, n_points))

    # Compute RMSE per joint and total
    rmse = np.sqrt(np.mean((planned_resampled - executed_resampled) ** 2, axis=0))
    total_rmse = np.sqrt(np.mean((planned_resampled - executed_resampled) ** 2))
    print(f"Total RMSE of planned and executed trajectory: {total_rmse:.6f}")

    # Plot in same style as reduced joint trajectory
    fig, axs = plt.subplots(
        len(joint_pos_names), 1, figsize=(10, 2.5 * len(joint_pos_names)), sharex=True
    )

    if len(joint_pos_names) == 1:
        axs = [axs]

    for i, joint in enumerate(joint_pos_names):
        axs[i].plot(
            planned_resampled[:, i],
            marker="o",
            markersize=5,
            color="blue",
            alpha=0.5,
            label="Planned",
        )
        axs[i].plot(
            executed_resampled[:, i],
            marker="o",
            markersize=5,
            color="red",
            alpha=0.5,
            label="Executed",
        )
        axs[i].set_ylabel("Angle in radians")
        axs[i].set_title(f"{joint} (RMSE: {rmse[i]:.4f})")
        axs[i].set_ylim(-3.5, 3.5)
        axs[i].grid(True)

    axs[-1].set_xlabel("Normalized Trajectory Index")

    # Global legend
    fig.legend(
        ["Planned", "Executed"],
        loc="lower right",
        bbox_to_anchor=(1, 0),
        bbox_transform=fig.transFigure,
        ncol=2,
    )

    # Add total RMSE text below the last plot
    fig.text(0.5, 0.01, f"Total RMSE: {total_rmse:.6f}", ha="center", fontsize=14, style="italic")

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # Save figure
    base_name = os.path.basename(filepath_planned).replace(
        "_planned.csv", "_compare_planned_vs_executed.png"
    )
    plot_path = os.path.join(os.path.dirname(filepath_planned), base_name)
    plt.savefig(plot_path)
    plt.show()
    print(f"Figure with comparison saved to: {plot_path}")


def compare_and_plot_cartesian_trajectories(
    filepath_planned, filepath_executed, cart_pos_names, n_points
):
    # Load CSV files
    df_planned = pd.read_csv(filepath_planned)
    df_executed = pd.read_csv(filepath_executed)

    # Use only the first three values (x, y, z)
    pos_names = cart_pos_names[:3]

    # Extract position values
    planned_positions = df_planned[pos_names].values
    executed_positions = df_executed[pos_names].values

    # Resample planned trajectory (linear interpolation)
    interp_planned = interp1d(np.linspace(0, 1, len(planned_positions)), planned_positions, axis=0)
    planned_resampled = interp_planned(np.linspace(0, 1, n_points))

    # Resample executed trajectory
    interp_executed = interp1d(
        np.linspace(0, 1, len(executed_positions)), executed_positions, axis=0
    )
    executed_resampled = interp_executed(np.linspace(0, 1, n_points))

    # Calculate 3D RMSE
    diffs = planned_resampled - executed_resampled
    squared_distances = np.sum(diffs**2, axis=1)
    rmse_3d = np.sqrt(np.mean(squared_distances))
    print(f"3D RMSE of planned and executed trajectory: {rmse_3d:.6f}")

    # 3D Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        planned_resampled[:, 0],
        planned_resampled[:, 1],
        planned_resampled[:, 2],
        "o-",
        color="blue",
        alpha=0.6,
        label="Planned",
        markersize=4,
    )
    ax.plot(
        executed_resampled[:, 0],
        executed_resampled[:, 1],
        executed_resampled[:, 2],
        "o-",
        color="red",
        alpha=0.6,
        label="Executed",
        markersize=4,
    )

    ax.set_xlabel("X in m")
    ax.set_ylabel("Y in m")
    ax.set_zlabel("Z in m")

    x_limits = [
        np.min(np.concatenate([planned_resampled[:, 0], executed_resampled[:, 0]])),
        np.max(np.concatenate([planned_resampled[:, 0], executed_resampled[:, 0]])),
    ]
    y_limits = [
        np.min(np.concatenate([planned_resampled[:, 1], executed_resampled[:, 1]])),
        np.max(np.concatenate([planned_resampled[:, 1], executed_resampled[:, 1]])),
    ]
    z_limits = [
        np.min(np.concatenate([planned_resampled[:, 2], executed_resampled[:, 2]])),
        np.max(np.concatenate([planned_resampled[:, 2], executed_resampled[:, 2]])),
    ]

    # Determine common limit (min and max over all axes)
    min_limit = min(x_limits[0], y_limits[0], z_limits[0])
    max_limit = max(x_limits[1], y_limits[1], z_limits[1])

    ax.set_xlim(min_limit, max_limit)
    ax.set_ylim(min_limit, max_limit)
    ax.set_zlim(min_limit, max_limit)

    # ax.set_title('Cartesian Trajectories Comparison')
    fig.text(0.5, 0.01, f"RMSE: {rmse_3d:.6f}", ha="center", fontsize=14, style="italic")
    ax.legend()
    ax.grid(True)

    # Save figure
    base_name = os.path.basename(filepath_planned).replace(
        "_planned.csv", "_compare_cartesian_planned_vs_executed.png"
    )
    plot_path = os.path.join(os.path.dirname(filepath_planned), base_name)
    plt.savefig(plot_path)
    plt.show()
    print(f"3D figure with cartesian trajectory comparison saved to: {plot_path}")


def main():
    data_dir = "src/motion_primitives_forward_controller/data"
    filename_planned = "trajectory_<date>_planned.csv"
    filename_executed = "trajectory_<date>_executed.csv"
    filepath_planned = os.path.join(data_dir, filename_planned)
    filepath_executed = os.path.join(data_dir, filename_executed)
    joint_pos_names = [
        "shoulder_pan_joint_pos",
        "shoulder_lift_joint_pos",
        "elbow_joint_pos",
        "wrist_1_joint_pos",
        "wrist_2_joint_pos",
        "wrist_3_joint_pos",
    ]
    n_points = 100

    compare_and_plot_joint_trajectories(
        filepath_planned, filepath_executed, joint_pos_names, n_points
    )


if __name__ == "__main__":
    main()
