#!/usr/bin/env python3
"""Compare KISS-ICP LiDAR odometry against wheel odometry from a Scout 2.0 rosbag.

Reads a ROS 2 bag directly (no ROS install needed), runs KISS-ICP on the LiDAR
scans, extracts the wheel odometry from the same bag, and writes:

    trajectories.png    top-down view of both trajectories
    divergence.png      distance between the two estimates over time
    summary.txt         path lengths, end points, and divergence statistics
    kiss_icp_poses.csv  KISS-ICP trajectory (time, x, y, z, yaw)
    wheel_odom.csv      wheel odometry trajectory (time, x, y, yaw)

Neither estimate is ground truth. Where they disagree, the summary reports by
how much, not which is correct. If the run ended on its start mark, each
method's end-to-start distance is a direct measure of its own drift.

Setup (once):
    pip install kiss-icp rosbags matplotlib numpy

Usage:
    python kiss_vs_wheel.py PATH_TO_BAG_FOLDER
    python kiss_vs_wheel.py PATH_TO_BAG_FOLDER --max-range 60 --out results
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_from_matrix(T):
    return math.atan2(T[1, 0], T[0, 0])


def cloud_to_arrays(msg, read_points):
    """Points (N, 3) and per-point times normalised to [0, 1], NaNs removed."""
    t_field = None
    for field in msg.fields:
        if field.name in ("t", "timestamp", "time"):
            t_field = field.name
            break
    names = ["x", "y", "z"] + ([t_field] if t_field else [])
    pts = read_points(msg, field_names=names)
    xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
    keep = np.all(np.isfinite(xyz), axis=1) & np.any(xyz != 0.0, axis=1)
    xyz = xyz[keep]
    if not t_field:
        return xyz, np.array([])
    t = pts[t_field].astype(np.float64)[keep]
    span = t.max() - t.min() if t.size else 0.0
    if span <= 0.0:
        return xyz, np.array([])
    return xyz, (t - t.min()) / span


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def path_length(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0


def best_rotation_2d(a, b):
    """Rotation angle that best maps point set a onto b, both anchored at the origin."""
    num = np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
    den = np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])
    return math.atan2(num, den)


def main():
    ap = argparse.ArgumentParser(description="KISS-ICP vs. wheel odometry on a Scout 2.0 rosbag.")
    ap.add_argument("bag", type=Path, help="bag folder, containing the .db3 and metadata.yaml")
    ap.add_argument("--lidar-topic", default="/rslidar_points")
    ap.add_argument("--odom-topic", default="/agilex_odom")
    ap.add_argument("--max-range", type=float, default=60.0, help="ignore LiDAR points beyond this, metres")
    ap.add_argument("--min-range", type=float, default=0.5, help="ignore LiDAR points closer than this, metres (robot body)")
    ap.add_argument("--align-distance", type=float, default=15.0,
                    help="metres of travel at the start used to align the two trajectories")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default: results_<bag name>)")
    args = ap.parse_args()

    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
        from kiss_icp.config import load_config
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.tools.point_cloud2 import read_points
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as e:
        sys.exit(f"Missing package: {e.name}. Run: pip install kiss-icp rosbags matplotlib numpy")

    bag = args.bag.expanduser().resolve()
    if not (bag / "metadata.yaml").exists():
        sys.exit(f"{bag} does not look like a ROS 2 bag folder (no metadata.yaml).")
    out = args.out or Path(f"results_{bag.name}")
    out.mkdir(parents=True, exist_ok=True)

    config = load_config(None)
    config.data.max_range = args.max_range
    config.data.min_range = args.min_range
    config.mapping.voxel_size = args.max_range / 100.0
    odom = KissICP(config=config)

    lidar, wheel = [], []
    t_start = time.time()
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        topics = {c.topic for c in reader.connections}
        for needed in (args.lidar_topic, args.odom_topic):
            if needed not in topics:
                sys.exit(f"Topic {needed} not in bag. Available: {sorted(topics)}")
        conns = [c for c in reader.connections if c.topic in (args.lidar_topic, args.odom_topic)]
        n_scans = sum(c.msgcount for c in conns if c.topic == args.lidar_topic)
        print(f"Bag: {bag.name}, {n_scans} LiDAR scans. Running KISS-ICP...")

        deskew_reported = False
        for conn, stamp, raw in reader.messages(connections=conns):
            t = stamp / 1e9
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == args.odom_topic:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                wheel.append((t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))
                continue
            xyz, times = cloud_to_arrays(msg, read_points)
            if not deskew_reported:
                print("Per-point timestamps found: motion compensation on." if times.size
                      else "No per-point timestamps: motion compensation off.")
                deskew_reported = True
            if len(xyz) < 100:
                continue
            odom.register_frame(xyz, times)
            T = odom.last_pose
            lidar.append((t, T[0, 3], T[1, 3], T[2, 3], yaw_from_matrix(T)))
            if len(lidar) % 250 == 0:
                rate = len(lidar) / (time.time() - t_start)
                left = (n_scans - len(lidar)) / rate if rate else 0
                print(f"  {len(lidar)}/{n_scans} scans, {rate:.1f} scans/s, about {left / 60:.1f} min left")

    if len(lidar) < 2 or len(wheel) < 2:
        sys.exit("Not enough LiDAR or odometry messages to compare.")

    L = np.array(lidar)
    W = np.array(wheel)

    # Express wheel odometry relative to its first pose, so both start at (0, 0) facing +x.
    x0, y0, h0 = W[0, 1], W[0, 2], W[0, 3]
    c, s = math.cos(-h0), math.sin(-h0)
    dx, dy = W[:, 1] - x0, W[:, 2] - y0
    W_xy = np.column_stack([c * dx - s * dy, s * dx + c * dy])
    W_yaw = np.unwrap(W[:, 3] - h0)

    # KISS-ICP starts at the identity, in the LiDAR's own frame. Its heading change is
    # already the robot's heading change; only the x-y positions need rotating. Interpolate wheel
    # odometry to the LiDAR times, then find the fixed rotation (the LiDAR's mounting
    # yaw relative to the base) that best lines the two up. No scaling, no translation.
    L_xy_raw = L[:, 1:3]
    Wi = np.column_stack([np.interp(L[:, 0], W[:, 0], W_xy[:, 0]),
                          np.interp(L[:, 0], W[:, 0], W_xy[:, 1])])
    # Fit the mounting rotation on the start of the run only, before either method has
    # drifted much. Fitting on the whole run lets wheel-odometry heading drift bias it.
    travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Wi, axis=0), axis=1))])
    first = travelled <= args.align_distance
    if np.count_nonzero(first & (travelled > 1.0)) < 10:
        first = np.ones_like(first)
    mount_yaw = best_rotation_2d(L_xy_raw[first], Wi[first])
    cr, sr = math.cos(mount_yaw), math.sin(mount_yaw)
    L_xy = np.column_stack([cr * L_xy_raw[:, 0] - sr * L_xy_raw[:, 1],
                            sr * L_xy_raw[:, 0] + cr * L_xy_raw[:, 1]])

    gap = np.linalg.norm(L_xy - Wi, axis=1)
    tl = L[:, 0] - L[0, 0]
    len_l, len_w = path_length(L_xy), path_length(W_xy)
    end_l, end_w = L_xy[-1], W_xy[-1]

    # ---- outputs
    with open(out / "kiss_icp_poses.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "x_m", "y_m", "z_m", "yaw_rad"])
        for row, xy in zip(L, L_xy):
            wr.writerow([f"{row[0]:.6f}", f"{xy[0]:.4f}", f"{xy[1]:.4f}", f"{row[3]:.4f}", f"{wrap(row[4]):.5f}"])
    with open(out / "wheel_odom.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "x_m", "y_m", "yaw_rad"])
        for row, xy, yw in zip(W, W_xy, W_yaw):
            wr.writerow([f"{row[0]:.6f}", f"{xy[0]:.4f}", f"{xy[1]:.4f}", f"{yw:.5f}"])

    summary = [
        f"Bag: {bag.name}",
        f"Duration: {tl[-1]:.1f} s, {len(L)} LiDAR scans, {len(W)} wheel odometry messages",
        f"KISS-ICP settings: max range {args.max_range} m, min range {args.min_range} m, "
        f"voxel size {config.mapping.voxel_size:.2f} m",
        f"LiDAR mounting yaw used to align the trajectories: {math.degrees(mount_yaw):.1f} deg "
        f"(fitted on the first {args.align_distance:.0f} m)",
        "",
        f"Path length, KISS-ICP:        {len_l:8.2f} m",
        f"Path length, wheel odometry:  {len_w:8.2f} m",
        f"Difference:                   {len_w - len_l:+8.2f} m ({100 * (len_w - len_l) / len_l:+.1f}% of KISS-ICP)"
        if len_l > 0 else "",
        "",
        f"End point relative to start, KISS-ICP:       x {end_l[0]:7.2f} m, y {end_l[1]:7.2f} m, "
        f"distance {np.linalg.norm(end_l):.2f} m",
        f"End point relative to start, wheel odometry: x {end_w[0]:7.2f} m, y {end_w[1]:7.2f} m, "
        f"distance {np.linalg.norm(end_w):.2f} m",
        "If the run ended on its start mark, each distance above is that method's accumulated drift.",
        "",
        f"Distance between the two estimates: mean {gap.mean():.2f} m, max {gap.max():.2f} m "
        f"(at t = {tl[gap.argmax()]:.0f} s), final {gap[-1]:.2f} m",
        f"Final heading difference: {math.degrees(wrap(L[-1, 4] - W_yaw[-1])):+.1f} deg",
        "",
        "Neither estimate is ground truth. KISS-ICP has no loop closure, so it also drifts.",
        "Self-consistency check: where the robot drove the same corridor twice, a correct",
        "estimate draws the two passes on top of each other in trajectories.png.",
    ]
    (out / "summary.txt").write_text("\n".join(line for line in summary) + "\n")

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(W_xy[:, 0], W_xy[:, 1], color="#2b6cb0", lw=2, label="Wheel odometry")
    ax.plot(L_xy[:, 0], L_xy[:, 1], color="#c05621", lw=2, label="KISS-ICP (LiDAR)")
    ax.plot(0, 0, "o", color="black", ms=8, label="Start")
    ax.plot(*end_w, "s", color="#2b6cb0", ms=8, label="End, wheel")
    ax.plot(*end_l, "s", color="#c05621", ms=8, label="End, KISS-ICP")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{bag.name}: LiDAR odometry vs. wheel odometry")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=150)

    fig2, ax2 = plt.subplots(figsize=(9, 3.5))
    ax2.plot(tl, gap, color="#4a5568", lw=1.5)
    ax2.set_xlabel("Time since start (s)")
    ax2.set_ylabel("Distance between estimates (m)")
    ax2.set_title(f"{bag.name}: divergence between KISS-ICP and wheel odometry")
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(out / "divergence.png", dpi=150)

    print()
    print((out / "summary.txt").read_text())
    print(f"Results written to {out.resolve()}")


if __name__ == "__main__":
    main()
