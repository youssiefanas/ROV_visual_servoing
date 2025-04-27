import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re

CSV_NAME = "_bluerov2_visual_tracker.csv"  # Target CSV

def parse_vector3(s):
    """Extract x, y, z floats from a string like 'geometry_msgs.msg.Vector3(x=..., y=..., z=...)'."""
    match = re.findall(r"x=([-\d.]+), y=([-\d.]+), z=([-\d.]+)", s)
    if match:
        x, y, z = map(float, match[0])
        return x, y, z
    else:
        return np.nan, np.nan, np.nan

def plot_linear_y_angular_z(df, save_base):
    """Plot only linear_y and angular_z components over time and save as PDF and PNG."""
    plt.figure(figsize=(12, 7))

    plt.plot(df["Time"], df["linear_y"], label="Linear Y", linewidth=2)
    plt.plot(df["Time"], df["angular_z"], label="Angular Z", linewidth=2)

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Value", fontsize=20)
    # plt.title("Visual Tracker: Linear Y and Angular Z Over Time", fontsize=18)
    plt.legend(fontsize=16)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=18)  # Axis numbers font size
    plt.tight_layout()

    plt.savefig(f"{save_base}.pdf")
    plt.savefig(f"{save_base}.png", dpi=300)
    plt.close()
    print(f"Saved plots: {save_base}.pdf and {save_base}.png")

def process_visual_tracker_csv(csv_path):
    """Read and process visual tracker CSV, extracting y for linear and z for angular."""
    df = pd.read_csv(csv_path)
    df[["linear_x", "linear_y", "linear_z"]] = df["_linear"].apply(lambda s: pd.Series(parse_vector3(s)))
    df[["angular_x", "angular_y", "angular_z"]] = df["_angular"].apply(lambda s: pd.Series(parse_vector3(s)))
    df["Time"] = (df["timestamp"] - df["timestamp"].min()) / 1e9
    return df

def find_folders_with_csv(base_dir, csv_name):
    """Find all subfolders with the target CSV file."""
    folders = []
    for root, _, files in os.walk(base_dir):
        if csv_name in files:
            folders.append(root)
    return folders

def process_all_folders(base_dir):
    folders = find_folders_with_csv(base_dir, CSV_NAME)
    for folder in folders:
        csv_path = os.path.join(folder, CSV_NAME)
        plot_dir = os.path.join(folder, "plot")
        os.makedirs(plot_dir, exist_ok=True)
        plot_base = os.path.join(plot_dir, "visual_tracker_linearY_angularZ")
        try:
            df = process_visual_tracker_csv(csv_path)
            plot_linear_y_angular_z(df, plot_base)
        except Exception as e:
            print(f"Error in {folder}: {e}")

if __name__ == '__main__':
    base_directory = os.getcwd()
    process_all_folders(base_directory)
