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

def plot_errors_and_current(df, save_base):
    """Plot error (desired-current) x,y and current x,y vs time."""
    plt.figure(figsize=(12, 7))

    plt.plot(df["Time"], df["error_x"], label="Error X (desired-current)", linewidth=2)
    plt.plot(df["Time"], df["error_y"], label="Error Y (desired-current)", linewidth=2)
    plt.plot(df["Time"], df["current_x"], label="Current X", linewidth=2, linestyle='dashed')
    plt.plot(df["Time"], df["current_y"], label="Current Y", linewidth=2, linestyle='dashed')

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Value", fontsize=20)
    #plt.title("Tracked Point Error and Current XY over Time", fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=16)  # <-- This sets axis number font size

    plt.legend(fontsize=16)
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(f"{save_base}.pdf")
    plt.savefig(f"{save_base}.png", dpi=300)
    plt.close()
    print(f"Saved plots: {save_base}.pdf and {save_base}.png")

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
        plot_base = os.path.join(plot_dir, "tracked_point")
        try:
            df = process_tracked_point_csv(csv_path)
            plot_errors_and_current(df, plot_base)
        except Exception as e:
            print(f"Error in {folder}: {e}")

if __name__ == '__main__':
    base_directory = os.getcwd()
    process_all_folders(base_directory)
