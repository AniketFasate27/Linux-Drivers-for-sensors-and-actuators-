#!/usr/bin/env python3
"""
EECE5554 Lab 1 Plots (ROS 2 /gps from emulator or real puck)

Usage:
  python3 lab1_plots.py \
    --open   ~/EECE5554/gnss/data/open_stationary_2025... \
    --occ    ~/EECE5554/gnss/data/occluded_stationary_2025... \
    --move   ~/EECE5554/gnss/data/moving_2025... \
    --outdir ~/EECE5554/gnss/analysis/plots

Requires:
  - ROS 2 (Jazzy or similar) environment sourced
  - rosbag2_py python bindings available
  - gps_interface.msg.Customgps on the PYTHONPATH via your workspace install
"""

import argparse
import os
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

# ROS 2 bag reading
import rosbag2_py
from rclpy.serialization import deserialize_message
from gps_interface.msg import Customgps

# -----------------------------
# Helpers
# -----------------------------
def get_storage_id(bag_dir: str) -> str:
    """Read metadata.yaml to determine storage_id ('sqlite3' or 'mcap')."""
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    if not os.path.exists(meta_path):
        # Try newer layout (metadata.yaml at parent if there is a split)
        raise FileNotFoundError(f"metadata.yaml not found in {bag_dir}")
    with open(meta_path, "r") as f:
        meta = yaml.safe_load(f)
    # ROS 2 metadata may nest this field
    storage_id = meta.get("rosbag2_bagfile_information", {}).get("storage_identifier")
    if not storage_id:
        # Fallback for older formats
        storage_id = meta.get("storage_identifier", "sqlite3")
    return storage_id

def read_bag_to_arrays(bag_dir: str, topic_name: str = "/gps"):
    """Read a rosbag2 directory and return dict of numpy arrays for needed fields."""
    storage_id = get_storage_id(bag_dir)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    topic_names = [t.name for t in topics]
    if topic_name not in topic_names:
        print(f"[WARN] Topic '{topic_name}' not found in {bag_dir}. Available: {topic_names}")

    times, east, north, alt, lat, lon, hdop = [], [], [], [], [], [], []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, Customgps)
        # time in seconds (float)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        times.append(ts)
        east.append(msg.utm_easting)
        north.append(msg.utm_northing)
        alt.append(msg.altitude)
        lat.append(msg.latitude)
        lon.append(msg.longitude)
        hdop.append(msg.hdop)

    if not times:
        raise RuntimeError(f"No messages read from '{topic_name}' in {bag_dir}")

    return {
        "t": np.array(times, dtype=float),
        "E": np.array(east, dtype=float),
        "N": np.array(north, dtype=float),
        "alt": np.array(alt, dtype=float),
        "lat": np.array(lat, dtype=float),
        "lon": np.array(lon, dtype=float),
        "hdop": np.array(hdop, dtype=float),
    }

def center_xy(E: np.ndarray, N: np.ndarray):
    """Return centered coordinates and the centroid (E0, N0)."""
    E0 = float(np.nanmean(E))
    N0 = float(np.nanmean(N))
    return (E - E0, N - N0, E0, N0)

def ensure_outdir(path: str):
    os.makedirs(path, exist_ok=True)
    return os.path.abspath(path)

# -----------------------------
# Plotting
# -----------------------------
def plot_stationary_EN(open_ds, occ_ds, outdir):
    # Raw scatter on same figure
    plt.figure(figsize=(7.5, 6.5))
    plt.scatter(open_ds["E"], open_ds["N"], s=8, label="Open (stationary)", alpha=0.7)
    plt.scatter(occ_ds["E"], occ_ds["N"], s=8, label="Occluded (stationary)", alpha=0.7)
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title("Stationary Northing vs. Easting (Raw)")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    raw_path = os.path.join(outdir, "stationary_EN_scatter.png")
    plt.savefig(raw_path, dpi=160)
    plt.close()

    # Centered scatter (both on same figure)
    Eo_c, No_c, Eo0, No0 = center_xy(open_ds["E"], open_ds["N"])
    Eocc_c, Nocc_c, Eocc0, Nocc0 = center_xy(occ_ds["E"], occ_ds["N"])

    plt.figure(figsize=(7.5, 6.5))
    plt.scatter(Eo_c, No_c, s=8, label=f"Open (centroid E0={Eo0:.2f}, N0={No0:.2f})", alpha=0.7)
    plt.scatter(Eocc_c, Nocc_c, s=8, label=f"Occluded (centroid E0={Eocc0:.2f}, N0={Nocc0:.2f})", alpha=0.7)
    plt.xlabel("Centered UTM Easting (m)")
    plt.ylabel("Centered UTM Northing (m)")
    plt.title("Stationary Northing vs. Easting (Centered by Centroid)")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    cen_path = os.path.join(outdir, "stationary_EN_scatter_centered.png")
    plt.savefig(cen_path, dpi=160)
    plt.close()

def plot_stationary_altitude(open_ds, occ_ds, outdir):
    # Normalize time to start at 0
    t_open = open_ds["t"] - open_ds["t"][0]
    t_occ  = occ_ds["t"]  - occ_ds["t"][0]

    plt.figure(figsize=(8.0, 5.8))
    plt.plot(t_open, open_ds["alt"], ".", label="Open (stationary)", markersize=3)
    plt.plot(t_occ,  occ_ds["alt"],  ".", label="Occluded (stationary)", markersize=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Stationary Altitude vs. Time")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(outdir, "stationary_altitude_vs_time.png")
    plt.savefig(out, dpi=160)
    plt.close()

def plot_stationary_distance_hist(open_ds, occ_ds, outdir, bins=30):
    # Distances from centroid
    Eo_c, No_c, _, _ = center_xy(open_ds["E"], open_ds["N"])
    Eocc_c, Nocc_c, _, _ = center_xy(occ_ds["E"], occ_ds["N"])
    r_open = np.sqrt(Eo_c**2 + No_c**2)
    r_occ  = np.sqrt(Eocc_c**2 + Nocc_c**2)

    # Use common bin edges so histograms are comparable
    r_all = np.concatenate([r_open, r_occ])
    bins_edges = np.histogram_bin_edges(r_all, bins=bins)

    fig = plt.figure(figsize=(10, 4.2))
    # Left subplot: Open
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.hist(r_open, bins=bins_edges, edgecolor="black", alpha=0.9)
    ax1.set_xlabel("Distance from centroid (m)")
    ax1.set_ylabel("Count")
    ax1.set_title("Open (stationary)")
    ax1.grid(True, linestyle="--", alpha=0.3)

    # Right subplot: Occluded
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.hist(r_occ, bins=bins_edges, edgecolor="black", alpha=0.9)
    ax2.set_xlabel("Distance from centroid (m)")
    ax2.set_title("Occluded (stationary)")
    ax2.grid(True, linestyle="--", alpha=0.3)

    fig.suptitle("Stationary Euclidean Distance from Centroid (Histograms)", y=1.02, fontsize=12)
    fig.tight_layout()
    out = os.path.join(outdir, "stationary_r_hist_subfigs.png")
    fig.savefig(out, dpi=160, bbox_inches="tight")
    plt.close(fig)

def plot_moving_EN_and_fit(move_ds, outdir):
    E = move_ds["E"]
    N = move_ds["N"]
    # Best fit line N = a*E + b
    a, b = np.polyfit(E, N, 1)

    plt.figure(figsize=(7.5, 6.5))
    plt.plot(E, N, ".", markersize=3, label="Trajectory")
    # Fit line across the span of E
    Ef = np.linspace(E.min(), E.max(), 100)
    Nf = a * Ef + b
    plt.plot(Ef, Nf, "-", linewidth=2, label=f"Best fit: N = {a:.4f}*E + {b:.2f}")
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.title("Moving Data: Northing vs. Easting with Best-Fit Line")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(outdir, "moving_EN_with_fit.png")
    plt.savefig(out, dpi=160)
    plt.close()

def plot_moving_altitude(move_ds, outdir):
    t = move_ds["t"] - move_ds["t"][0]
    plt.figure(figsize=(8.0, 5.8))
    plt.plot(t, move_ds["alt"], ".", markersize=3, label="Moving")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Moving Data: Altitude vs. Time")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(outdir, "moving_altitude_vs_time.png")
    plt.savefig(out, dpi=160)
    plt.close()

# -----------------------------
# Main
# -----------------------------
def main():
    parser = argparse.ArgumentParser(description="EECE5554 Lab 1 plotting from /gps rosbags.")
    parser.add_argument("--open", required=True, help="Path to OPEN stationary bag folder")
    parser.add_argument("--occ",  required=True, help="Path to OCCLUDED stationary bag folder")
    parser.add_argument("--move", required=True, help="Path to MOVING bag folder")
    parser.add_argument("--outdir", default=os.path.expanduser("~/EECE5554/gnss/analysis/plots"),
                        help="Directory to write plots (will be created)")
    args = parser.parse_args()

    outdir = ensure_outdir(args.outdir)

    print("[INFO] Reading OPEN stationary bag...")
    open_ds = read_bag_to_arrays(args.open)

    print("[INFO] Reading OCCLUDED stationary bag...")
    occ_ds = read_bag_to_arrays(args.occ)

    print("[INFO] Reading MOVING bag...")
    move_ds = read_bag_to_arrays(args.move)

    # Stationary EN scatter (raw + centered)
    print("[INFO] Making stationary E/N scatter plots...")
    plot_stationary_EN(open_ds, occ_ds, outdir)

    # Stationary altitude vs time
    print("[INFO] Making stationary altitude vs time plot...")
    plot_stationary_altitude(open_ds, occ_ds, outdir)

    # Stationary histograms of distance from centroid (subfigs)
    print("[INFO] Making stationary distance histograms...")
    plot_stationary_distance_hist(open_ds, occ_ds, outdir, bins=30)

    # Moving EN with best-fit line
    print("[INFO] Making moving E/N with best-fit line...")
    plot_moving_EN_and_fit(move_ds, outdir)

    # Moving altitude vs time
    print("[INFO] Making moving altitude vs time...")
    plot_moving_altitude(move_ds, outdir)

    print(f"[DONE] Plots saved in: {outdir}")

if __name__ == "__main__":
    main()
