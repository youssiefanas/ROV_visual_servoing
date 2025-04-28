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

def plot_linear_y_angular_z(df, bag_name):
    """Plot only linear_y and angular_z components over time and save as PDF and PNG in correct folders."""
    base_dir = os.getcwd()
    pdf_dir = os.path.join(base_dir, "plot_result", bag_name, "pdf")
    png_dir = os.path.join(base_dir, "plot_result", bag_name, "png")
    os.makedirs(pdf_dir, exist_ok=True)
    os.makedirs(png_dir, exist_ok=True)
    pdf_path = os.path.join(pdf_dir, "visual_tracker_linearY_angularZ.pdf")
    png_path = os.path.join(png_dir, "visual_tracker_linearY_angularZ.png")

    plt.figure(figsize=(12, 7))
    plt.plot(df["Time"], df["linear_y"], label="Linear Y", linewidth=2)
    plt.plot(df["Time"], df["angular_z"], label="Angular Z", linewidth=2)

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Value", fontsize=20)
    plt.legend(fontsize=16)
    plt.grid(True)
    plt.tick_params(axis='both', which='major', labelsize=18)
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.savefig(png_path, dpi=300)
    plt.close()
    print(f"Saved plots: {pdf_path} and {png_path}")

def process_visual_tracker_csv(csv_path):
    """Read and process visual tracker CSV, extracting y for linear and z for angular."""
    df = pd.read_csv(csv_path)
    df[["linear_x", "linear_y", "linear_z"]] = df["_linear"].apply(lambda s: pd.Series(parse_vector3(s)))
    df[["angular_x", "angular_y", "angular_z"]] = df["_angular"].apply(lambda s: pd.Series(parse_vector3(s)))
    df["Time"] = (df["timestamp"] - df["timestamp"].min()) / 1e9
    return df

def find_visual_tracker_csvs(base_dir):
    """Find all _bluerov2_visual_tracker.csv files and their bag_name."""
    result = []
    for root, _, files in os.walk(base_dir):
        if CSV_NAME in files:
            # Expect path: <cwd>/rosbag_data/<bagname>/<CSV_NAME>
            path_parts = os.path.normpath(root).split(os.sep)
            try:
                bag_idx = path_parts.index('rosbag_data') + 1
                bag_name = path_parts[bag_idx]
            except Exception:
                bag_name = "unknown"
            csv_path = os.path.join(root, CSV_NAME)
            result.append((csv_path, bag_name))
    return result

def process_all_visual_tracker_csvs():
    base_dir = os.path.join(os.getcwd(), "rosbag_data")
    csvs = find_visual_tracker_csvs(base_dir)
    for csv_path, bag_name in csvs:
        try:
            df = process_visual_tracker_csv(csv_path)
            plot_linear_y_angular_z(df, bag_name)
        except Exception as e:
            print(f"Error processing {csv_path}: {e}")

if __name__ == '__main__':
    process_all_visual_tracker_csvs()
