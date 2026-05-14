#!/usr/bin/env python3
"""
Analyse a pico_bridge rosbag2 recording.

Plots:
  - combined_paths.png   : GPS raw, GPS odom, and EKF filtered xy paths
  - ekf_xy_time.png      : EKF x and y over time
  - imu_gyro_z_time.png  : IMU angular velocity z over time

Stats printed to stdout:
  - Total distance travelled (EKF and GPS-raw)
  - Max position excursion during the assumed-stationary boot window
  - Start vs end position error (EKF and GPS-raw)

Usage:
  python3 analyse_bag.py <bag_path> [--stationary-seconds 10]
"""

import argparse
import math
import os
import sys

import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


WGS84_A = 6378137.0  # equatorial radius (m), matches gps_to_odom_node


# ---------- bag reading ----------

TOPICS = ('/fix', '/odometry/filtered', '/odometry/gps', '/imu/data_raw')


def read_bag(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    out = {topic: [] for topic in TOPICS}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        msg_type_str = type_map.get(topic)
        if not msg_type_str:
            continue
        msg = deserialize_message(data, get_message(msg_type_str))
        out[topic].append((t_ns * 1e-9, msg))

    return out


# ---------- extraction ----------

def extract_fix_xy(fix_msgs):
    """Convert /fix lat/lon to local x/y using first valid fix as datum."""
    times, xs, ys = [], [], []
    datum = None
    for t, msg in fix_msgs:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            continue
        if msg.status.status < 0:  # STATUS_NO_FIX
            continue
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        if datum is None:
            datum = (lat, lon)
        x = WGS84_A * math.cos(datum[0]) * (lon - datum[1])
        y = WGS84_A * (lat - datum[0])
        times.append(t)
        xs.append(x)
        ys.append(y)
    return times, xs, ys


def extract_odom_xy(odom_msgs):
    times, xs, ys = [], [], []
    for t, msg in odom_msgs:
        times.append(t)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
    return times, xs, ys


def extract_imu_gyro_z(imu_msgs):
    times, gz = [], []
    for t, msg in imu_msgs:
        times.append(t)
        gz.append(msg.angular_velocity.z)
    return times, gz


# ---------- stats ----------

def total_distance(xs, ys):
    if len(xs) < 2:
        return 0.0
    d = 0.0
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        d += math.hypot(dx, dy)
    return d


def stationary_drift(times, xs, ys, window_s):
    """Max distance from initial position during the first `window_s` seconds."""
    if not times:
        return 0.0
    t0 = times[0]
    x0, y0 = xs[0], ys[0]
    max_d = 0.0
    for t, x, y in zip(times, xs, ys):
        if t - t0 > window_s:
            break
        d = math.hypot(x - x0, y - y0)
        if d > max_d:
            max_d = d
    return max_d


def start_end_error(xs, ys):
    if len(xs) < 2:
        return 0.0
    return math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])


# ---------- plots ----------

def plot_combined_paths(fix_xy, ekf_xy, gps_odom_xy, out_path):
    fig, ax = plt.subplots(figsize=(8, 8))

    if fix_xy[1]:
        ax.plot(fix_xy[1], fix_xy[2], '-o', markersize=2, label='GPS raw (/fix)', color='tab:green')
    if gps_odom_xy[1]:
        ax.plot(gps_odom_xy[1], gps_odom_xy[2], '-', label='GPS odom (/odometry/gps)', color='tab:orange')
    if ekf_xy[1]:
        ax.plot(ekf_xy[1], ekf_xy[2], '-', label='EKF (/odometry/filtered)', color='tab:blue')

    ax.set_xlabel('x (m, east)')
    ax.set_ylabel('y (m, north)')
    ax.set_title('XY trajectories')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_ekf_xy_time(ekf_xy, out_path):
    times, xs, ys = ekf_xy
    if not times:
        return
    t0 = times[0]
    rel = [t - t0 for t in times]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(rel, xs, color='tab:blue')
    axes[0].set_ylabel('x (m)')
    axes[0].set_title('EKF position over time')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(rel, ys, color='tab:red')
    axes[1].set_ylabel('y (m)')
    axes[1].set_xlabel('time (s)')
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_imu_gyro_z(imu_gyro_z, out_path):
    times, gz = imu_gyro_z
    if not times:
        return
    t0 = times[0]
    rel = [t - t0 for t in times]

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(rel, gz, color='tab:purple', linewidth=0.8)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('gyro z (rad/s)')
    ax.set_title('IMU angular velocity z')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='black', linewidth=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------- main ----------

def main():
    parser = argparse.ArgumentParser(description='Analyse a pico_bridge rosbag.')
    parser.add_argument('bag_path', help='Path to the rosbag2 directory.')
    parser.add_argument('--stationary-seconds', type=float, default=10.0,
                        help='Boot window assumed stationary, for drift stat (default: 10).')
    parser.add_argument('--out-dir', default='.',
                        help='Directory for output PNGs (default: cwd).')
    args = parser.parse_args()

    if not os.path.exists(args.bag_path):
        print(f'error: bag path does not exist: {args.bag_path}', file=sys.stderr)
        sys.exit(1)
    os.makedirs(args.out_dir, exist_ok=True)

    print(f'Reading bag: {args.bag_path}')
    msgs = read_bag(args.bag_path)
    for topic in TOPICS:
        print(f'  {topic}: {len(msgs[topic])} messages')

    fix_xy      = extract_fix_xy(msgs['/fix'])
    ekf_xy      = extract_odom_xy(msgs['/odometry/filtered'])
    gps_odom_xy = extract_odom_xy(msgs['/odometry/gps'])
    imu_gz      = extract_imu_gyro_z(msgs['/imu/data_raw'])

    combined_path  = os.path.join(args.out_dir, 'combined_paths.png')
    ekf_time_path  = os.path.join(args.out_dir, 'ekf_xy_time.png')
    imu_gyro_path  = os.path.join(args.out_dir, 'imu_gyro_z_time.png')

    plot_combined_paths(fix_xy, ekf_xy, gps_odom_xy, combined_path)
    plot_ekf_xy_time(ekf_xy, ekf_time_path)
    plot_imu_gyro_z(imu_gz, imu_gyro_path)
    print(f'Wrote {combined_path}')
    print(f'Wrote {ekf_time_path}')
    print(f'Wrote {imu_gyro_path}')

    # ---------- stats ----------
    print()
    print('=== Summary stats ===')

    if ekf_xy[0]:
        ekf_dist  = total_distance(ekf_xy[1], ekf_xy[2])
        ekf_drift = stationary_drift(ekf_xy[0], ekf_xy[1], ekf_xy[2], args.stationary_seconds)
        ekf_err   = start_end_error(ekf_xy[1], ekf_xy[2])
        print(f'EKF total distance travelled : {ekf_dist:8.2f} m')
        print(f'EKF max drift in first {args.stationary_seconds:.0f}s    : {ekf_drift:8.2f} m   (assumed stationary)')
        print(f'EKF start->end displacement  : {ekf_err:8.2f} m')
    else:
        print('EKF: no messages on /odometry/filtered')

    if fix_xy[0]:
        gps_dist  = total_distance(fix_xy[1], fix_xy[2])
        gps_drift = stationary_drift(fix_xy[0], fix_xy[1], fix_xy[2], args.stationary_seconds)
        gps_err   = start_end_error(fix_xy[1], fix_xy[2])
        print(f'GPS-raw total path length    : {gps_dist:8.2f} m')
        print(f'GPS-raw max drift in first {args.stationary_seconds:.0f}s: {gps_drift:8.2f} m   (assumed stationary)')
        print(f'GPS-raw start->end displacement: {gps_err:8.2f} m')
    else:
        print('GPS-raw: no messages on /fix')


if __name__ == '__main__':
    main()
