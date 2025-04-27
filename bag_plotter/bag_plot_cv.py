import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re

CSV_NAME = "_camera_velocity.csv"  # The CSV file to look for in subfolders

def parse_vector3(s):
    """Extract x, y, z floats from a string like 'geometry_msgs.msg.Vector3(x=..., y=..., z=...)'."""
    match = re.findall(r"x=([-\d.]+), y=([-\d.]+), z=([-\d.]+)", s)
    if match:
        x, y, z = map(float, match[0])
        return x, y, z
    else:
        return np.nan, np.nan, np.nan

def plot_vector3(df, save_base):
    """Generate and save the linear/angular velocity plot as PDF and PNG."""
    plt.figure(figsize=(12, 7))

    plt.plot(df["Time"], df["linear_x"], label="Linear X", linewidth=2)
    plt.plot(df["Time"], df["linear_y"], label="Linear Y", linewidth=2)
    plt.plot(df["Time"], df["linear_z"], label="Linear Z", linewidth=2)
    plt.plot(df["Time"], df["angular_x"], label="Angular X", linewidth=2, linestyle='dashed')
    plt.plot(df["Time"], df["angular_y"], label="Angular Y", linewidth=2, linestyle='dashed')
    plt.plot(df["Time"], df["angular_z"], label="Angular Z", linewidth=2, linestyle='dashed')

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Value", fontsize=20)
    #plt.title("Camera Linear and Angular Velocity Over Time", fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=16)  # <-- This sets axis number font size
    plt.legend(fontsize=15)
    plt.grid(True)
    plt.tight_layout()

    # Save plots
    plt.savefig(f"{save_base}.pdf")
    plt.savefig(f"{save_base}.png", dpi=300)
    plt.close()
    print(f"Saved plots: {save_base}.pdf and {save_base}.png")

def process_camera_velocity_csv(csv_path):
    """Read, process, and plot data from the camera velocity CSV."""
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
        os.makedirs(plot_dir, exist_ok=True)  # Create the 'plot' folder if it doesn't exist
        plot_base = os.path.join(plot_dir, "camera_velocity")  # Plots will be named "camera_velocity.pdf/png" inside 'plot'

        try:
            df = process_camera_velocity_csv(csv_path)
            plot_vector3(df, plot_base)
        except Exception as e:
            print(f"Error in {folder}: {e}")

if __name__ == '__main__':
    base_directory = os.getcwd()
    process_all_folders(base_directory)
