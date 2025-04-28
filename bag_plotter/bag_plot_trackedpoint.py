import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import ast

CSV_NAME = "_tracked_point.csv"  # The CSV file to look for in subfolders

def parse_array_d(array_str):
    """Parse the array('d', [...]) string to a Python list of floats."""
    match = re.search(r"array\('d',\s*\[([^\]]+)\]\)", array_str)
    if match:
        # Convert the string of numbers to a list
        values_str = "[" + match.group(1) + "]"
        try:
            return list(map(float, ast.literal_eval(values_str)))
        except Exception:
            return [np.nan] * 5
    return [np.nan] * 5

def plot_errors_and_current(df, bag_name):
    """Plot error (desired-current) x,y and current x,y vs time, saving under plot_result/<bagname>/pdf or png."""
    base_dir = os.getcwd()
    pdf_dir = os.path.join(base_dir, "plot_result", bag_name, "pdf")
    png_dir = os.path.join(base_dir, "plot_result", bag_name, "png")
    os.makedirs(pdf_dir, exist_ok=True)
    os.makedirs(png_dir, exist_ok=True)
    pdf_path = os.path.join(pdf_dir, "tracked_point.pdf")
    png_path = os.path.join(png_dir, "tracked_point.png")

    plt.figure(figsize=(12, 7))
    plt.plot(df["Time"], df["error_x"], label="Error X (desired-current)", linewidth=2)
    plt.plot(df["Time"], df["error_y"], label="Error Y (desired-current)", linewidth=2)
    plt.plot(df["Time"], df["current_x"], label="Current X", linewidth=2, linestyle='dashed')
    plt.plot(df["Time"], df["current_y"], label="Current Y", linewidth=2, linestyle='dashed')

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Value", fontsize=20)
    plt.tick_params(axis='both', which='major', labelsize=16)
    plt.legend(fontsize=16)
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(pdf_path)
    plt.savefig(png_path, dpi=300)
    plt.close()
    print(f"Saved plots: {pdf_path} and {png_path}")

def process_tracked_point_csv(csv_path):
    """Read, process, and return a DataFrame with columns for plotting."""
    df = pd.read_csv(csv_path)
    parsed = df["_data"].apply(parse_array_d)
    df["error_x"] = parsed.apply(lambda arr: arr[0] if len(arr) == 5 else np.nan)
    df["error_y"] = parsed.apply(lambda arr: arr[1] if len(arr) == 5 else np.nan)
    df["current_x"] = parsed.apply(lambda arr: arr[3] if len(arr) == 5 else np.nan)
    df["current_y"] = parsed.apply(lambda arr: arr[4] if len(arr) == 5 else np.nan)
    df["Time"] = (df["timestamp"] - df["timestamp"].min()) / 1e9
    return df

def find_tracked_point_csvs(base_dir):
    """Find all _tracked_point.csv files and their bag_name."""
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

def process_all_tracked_point_csvs():
    base_dir = os.path.join(os.getcwd(), "rosbag_data")
    csvs = find_tracked_point_csvs(base_dir)
    for csv_path, bag_name in csvs:
        try:
            df = process_tracked_point_csv(csv_path)
            plot_errors_and_current(df, bag_name)
        except Exception as e:
            print(f"Error processing {csv_path}: {e}")

if __name__ == '__main__':
    process_all_tracked_point_csvs()
