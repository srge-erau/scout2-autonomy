#!/usr/bin/env python3
"""Extract the signals the odometry benchmark needs from a Scout 2.0 rosbag.

Reads a ROS 2 bag directly (no ROS install needed) and writes, into results_<bag name>
(the same folder kiss_vs_wheel.py uses):

    wheel.csv          wheel odometry: pose, forward speed and yaw rate
    imu.csv            ZED IMU: gyro, accelerometer, and yaw rate about the vertical
    zed_odom.csv       ZED SDK odometry (the camera's own visual-inertial estimate)
    lidar_stamps.csv   record time and header stamp of every LiDAR scan
    camera_info.json   left camera intrinsics
    tf_static.json     static transforms recorded in the bag
    calibration.json   stationary window, gyro bias, gravity direction, noise levels,
                       and the clock check for each topic

Topics a bag does not contain are skipped. Order of the benchmark scripts:
kiss_vs_wheel.py, extract_signals.py, visual_odometry.py, fuse_ekf.py, evaluate.py.

Times: every CSV has time_s and record_s. record_s is the bag record time, as in
kiss_vs_wheel.py. time_s is the message header stamp (shifted onto the record clock if
the sensor stamps with its own clock). Header stamps keep the true spacing of samples;
record times do not, because messages reach the recorder late and then in bursts.

Setup (once):
    pip install rosbags numpy

Usage:
    python extract_signals.py PATH_TO_BAG_FOLDER
"""
import argparse
import csv
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rpy_from_quaternion(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch, yaw_from_quaternion(x, y, z, w)


def header_time(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def choose_time_base(record, head):
    """Header stamps when they are usable, else record times.

    Header stamps give the true sample spacing; record times jitter because messages
    reach the recorder late and then in bursts. If the header clock is not the record
    clock (offset of a second or more), the header stamps are shifted by the median
    offset so every topic stays on the record clock.
    """
    record, head = np.asarray(record), np.asarray(head)
    if len(record) < 2:
        return record, {"time_base": "record", "reason": "too few messages"}
    lat = record - head
    med = float(np.median(lat))
    backwards = float(np.mean(np.diff(head) <= 0.0))
    info = {"median_record_minus_header_s": round(med, 4),
            "p95_latency_spread_s": round(float(np.percentile(np.abs(lat - med), 95)), 4),
            "header_non_increasing_fraction": round(backwards, 4)}
    if backwards >= 0.01:
        info["time_base"] = "record"
        info["reason"] = "header stamps not increasing"
        return record, info
    if abs(med) < 1.0:
        info["time_base"] = "header"
        return head, info
    info["time_base"] = "header shifted to record clock"
    info["reason"] = f"header clock differs from record clock by {med:.1f} s"
    return head + med, info


def gyro_usable(rec, head, g, frozen_run=50):
    """Samples whose gyro reading can be trusted (same rule as fuse_ekf.py).

    Rejected: stamps more than 0.5 s off the usual record-minus-stamp lag (the stream was
    stale), all three axes exactly zero, and runs of at least `frozen_run` identical
    samples (a frozen sensor; real gyro noise changes every sample).
    """
    lat = rec - head
    ok = np.abs(lat - np.median(lat)) <= 0.5
    ok &= ~np.all(g == 0.0, axis=1)
    same = np.concatenate([[False], np.all(np.diff(g, axis=0) == 0.0, axis=1)])
    edges = np.flatnonzero(np.diff(np.concatenate([[0], same.astype(int), [0]])))
    for a, b in zip(edges[::2], edges[1::2]):
        if b - a + 1 >= frozen_run:
            ok[max(0, a - 1):b] = False
    return ok


def cdr_header_stamp(raw):
    """Header stamp read straight from a serialized ROS 2 message (CDR), without decoding it."""
    little = raw[1] == 1
    sec, nsec = struct.unpack_from("<iI" if little else ">iI", raw, 4)
    return sec + nsec * 1e-9


def rate_hz(t):
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 and t[-1] > t[0] else 0.0


def velocity_from_pose(t, x, y, yaw, window_s=0.1):
    """Forward speed and yaw rate from pose differences over about window_s."""
    yaw = np.unwrap(yaw)
    n = len(t)
    dt_med = float(np.median(np.diff(t))) if n > 1 else 0.02
    k = max(1, int(round(window_s / 2 / max(dt_med, 1e-3))))
    i0 = np.clip(np.arange(n) - k, 0, n - 1)
    i1 = np.clip(np.arange(n) + k, 0, n - 1)
    dt = np.maximum(t[i1] - t[i0], 1e-6)
    mid = 0.5 * (yaw[i0] + yaw[i1])
    v = ((x[i1] - x[i0]) * np.cos(mid) + (y[i1] - y[i0]) * np.sin(mid)) / dt
    w = (yaw[i1] - yaw[i0]) / dt
    return v, w


def main():
    ap = argparse.ArgumentParser(description="Extract wheel, IMU, ZED odometry and camera info from a Scout 2.0 rosbag.")
    ap.add_argument("bag", type=Path, help="bag folder, containing the .db3 and metadata.yaml")
    ap.add_argument("--odom-topic", default="/agilex_odom")
    ap.add_argument("--imu-topic", default="/zed/zed_node/imu/data")
    ap.add_argument("--zed-odom-topic", default="/zed/zed_node/odom")
    ap.add_argument("--camera-info-topic", default="/zed/zed_node/left/camera_info")
    ap.add_argument("--lidar-topic", default="/rslidar_points")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default: results_<bag name>)")
    args = ap.parse_args()

    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as e:
        sys.exit(f"Missing package: {e.name}. Run: pip install rosbags numpy")

    bag = args.bag.expanduser().resolve()
    if not (bag / "metadata.yaml").exists():
        sys.exit(f"{bag} does not look like a ROS 2 bag folder (no metadata.yaml).")
    out = args.out or Path(f"results_{bag.name}")
    out.mkdir(parents=True, exist_ok=True)

    wheel, imu, zodom, tf_static, lidar = [], [], [], [], []
    cam_info, n_cam_info = None, 0
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        available = {c.topic for c in reader.connections}
        wanted = [args.odom_topic, args.imu_topic, args.zed_odom_topic, args.camera_info_topic,
                  args.lidar_topic, "/tf_static"]
        for topic in wanted:
            print(f"  {'found  ' if topic in available else 'MISSING'} {topic}")
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            sys.exit(f"None of the expected topics are in this bag. Available: {sorted(available)}")
        print(f"Reading {bag.name} (the LiDAR scans are large; this can take a minute)...")
        for conn, stamp, raw in reader.messages(connections=conns):
            rec = stamp / 1e9
            topic = conn.topic
            if topic == args.lidar_topic:
                lidar.append((rec, cdr_header_stamp(raw)))
                continue
            if topic == args.camera_info_topic:
                n_cam_info += 1
                if cam_info is not None:
                    continue
            msg = reader.deserialize(raw, conn.msgtype)
            if topic == args.odom_topic:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                tw = msg.twist.twist
                wheel.append((rec, header_time(msg), p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                              tw.linear.x, tw.angular.z))
            elif topic == args.imu_topic:
                g, a = msg.angular_velocity, msg.linear_acceleration
                imu.append((rec, header_time(msg), g.x, g.y, g.z, a.x, a.y, a.z))
            elif topic == args.zed_odom_topic:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                zodom.append((rec, header_time(msg), p.x, p.y, p.z, yaw_from_quaternion(q.x, q.y, q.z, q.w)))
            elif topic == args.camera_info_topic:
                cam_info = {"frame_id": msg.header.frame_id, "width": int(msg.width), "height": int(msg.height),
                            "distortion_model": msg.distortion_model,
                            "k": [float(v) for v in msg.k], "d": [float(v) for v in msg.d],
                            "p": [float(v) for v in msg.p]}
            elif topic == "/tf_static":
                for tr in msg.transforms:
                    tt, rr = tr.transform.translation, tr.transform.rotation
                    roll, pitch, yaw = rpy_from_quaternion(rr.x, rr.y, rr.z, rr.w)
                    tf_static.append({"parent": tr.header.frame_id, "child": tr.child_frame_id,
                                      "translation_m": [tt.x, tt.y, tt.z],
                                      "rotation_xyzw": [rr.x, rr.y, rr.z, rr.w],
                                      "rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]})

    calib = {"bag": bag.name, "topics": {}}
    lines = [f"Bag: {bag.name}"]

    # ---- wheel odometry
    W = np.array(wheel) if wheel else np.zeros((0, 7))
    wt = None
    if len(W) > 1:
        wt, info = choose_time_base(W[:, 0], W[:, 1])
        v, w = W[:, 5].copy(), W[:, 6].copy()
        moved = float(np.sum(np.hypot(np.diff(W[:, 2]), np.diff(W[:, 3]))))
        twist_empty = np.max(np.abs(v)) < 1e-6 and np.max(np.abs(w)) < 1e-6
        if twist_empty and moved > 0.1:
            v, w = velocity_from_pose(wt, W[:, 2], W[:, 3], W[:, 4])
            info["speed_source"] = "pose differences (twist fields were empty)"
        else:
            info["speed_source"] = "twist fields"
        info.update(count=len(W), rate_hz=round(rate_hz(wt), 2))
        calib["topics"]["wheel"] = info
        W_yaw = np.unwrap(W[:, 4])
        with open(out / "wheel.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "record_s", "x_m", "y_m", "yaw_rad", "v_mps", "w_radps"])
            for i in range(len(W)):
                wr.writerow([f"{wt[i]:.6f}", f"{W[i, 0]:.6f}", f"{W[i, 2]:.4f}", f"{W[i, 3]:.4f}",
                             f"{W_yaw[i]:.5f}", f"{v[i]:.4f}", f"{w[i]:.5f}"])
        lines.append(f"Wheel odometry: {len(W)} messages at {info['rate_hz']} Hz, speed from {info['speed_source']}, "
                     f"time base {info['time_base']}")
        wheel_v, wheel_w = v, w
    else:
        lines.append("Wheel odometry: not in bag")

    # ---- stationary window at the start
    still = None
    if wt is not None:
        moving = (np.abs(wheel_v) > 0.02) | (np.abs(wheel_w) > 0.02)
        first_move = wt[np.argmax(moving)] if moving.any() else wt[-1]
        if first_move - 0.5 - wt[0] >= 1.0:
            still = (float(wt[0]), float(first_move - 0.5))
            calib["stationary_source"] = "wheel odometry"

    # ---- IMU
    I = np.array(imu) if imu else np.zeros((0, 8))
    if len(I) > 1:
        it, info = choose_time_base(I[:, 0], I[:, 1])
        gyro, acc = I[:, 2:5], I[:, 5:8]
        usable = gyro_usable(I[:, 0], I[:, 1], gyro)
        if still is None:
            # no wheel data: use the IMU, stationary until the gyro first exceeds 0.05 rad/s
            g0 = np.median(gyro[it < it[0] + 1.0], axis=0)
            mag = np.linalg.norm(gyro - g0, axis=1)
            idx = np.argmax(mag > 0.05) if (mag > 0.05).any() else len(it) - 1
            if it[idx] - 0.5 - it[0] >= 1.0:
                still = (float(it[0]), float(it[idx] - 0.5))
                calib["stationary_source"] = "IMU"
        if still is not None:
            m = (it >= still[0]) & (it <= still[1]) & usable
        else:
            m = np.zeros(len(it), bool)
        if m.sum() >= 50:
            up = acc[m].mean(axis=0)
            bias = gyro[m].mean(axis=0)
            gstd = gyro[m].std(axis=0)
            astd = acc[m].std(axis=0)
            calib["gravity_source"] = "stationary window"
        else:
            up = acc[usable].mean(axis=0) if usable.any() else acc.mean(axis=0)   # accelerations average out over a run
            bias = np.zeros(3)
            gstd = np.full(3, np.nan)
            astd = np.full(3, np.nan)
            calib["gravity_source"] = "mean over whole run (no stationary window)"
        g_mag = float(np.linalg.norm(up))
        u = up / g_mag
        yaw_rate = gyro @ u
        info.update(count=len(I), rate_hz=round(rate_hz(it[usable]) if usable.sum() > 1 else rate_hz(it), 2),
                    usable_pct=round(100.0 * float(usable.mean()), 1))
        if not usable.all():
            info["first_unusable_s"] = round(float(I[np.argmin(usable), 0] - I[0, 0]), 1)
            info["unusable_samples"] = int((~usable).sum())
        calib["topics"]["imu"] = info
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, u[2]))))
        calib.update(gravity_unit_imu=[float(c) for c in u], gravity_magnitude_mps2=round(g_mag, 4),
                     imu_tilt_from_vertical_deg=round(tilt, 2),
                     gyro_bias_radps=[float(c) for c in bias],
                     gyro_bias_up_radps=float(bias @ u),
                     gyro_std_radps=[float(c) for c in gstd], accel_std_mps2=[float(c) for c in astd],
                     yaw_rate_std_still_radps=float(np.std(yaw_rate[m])) if m.sum() >= 50 else None)
        with open(out / "imu.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "record_s", "gx_radps", "gy_radps", "gz_radps",
                         "ax_mps2", "ay_mps2", "az_mps2", "yaw_rate_radps"])
            for i in range(len(I)):
                wr.writerow([f"{it[i]:.6f}", f"{I[i, 0]:.6f}"] + [f"{c:.6f}" for c in I[i, 2:8]] + [f"{yaw_rate[i]:.6f}"])
        lines.append(f"IMU: {len(I)} messages at {info['rate_hz']} Hz, time base {info['time_base']}")
        if usable.mean() < 0.99:
            lines.append(f"  WARNING: {int((~usable).sum())} IMU samples unusable (stale stamps, zeros or frozen values), "
                         f"first at {info['first_unusable_s']:.1f} s of {I[-1, 0] - I[0, 0]:.1f} s; "
                         f"{info['usable_pct']:.0f}% usable. fuse_ekf.py uses only the usable samples.")
        else:
            lines.append(f"  usable samples: {info['usable_pct']:.1f}%")
        lines.append(f"  gravity {g_mag:.3f} m/s^2 ({calib['gravity_source']}), IMU z axis {tilt:.1f} deg from vertical")
        lines.append(f"  gyro bias about vertical {math.degrees(bias @ u):+.4f} deg/s")
    else:
        lines.append("IMU: not in bag")

    if still is not None:
        calib["stationary_window_s"] = [still[0], still[1]]
        lines.append(f"Stationary at start: {still[1] - still[0]:.1f} s (from {calib['stationary_source']})")
    else:
        calib["stationary_window_s"] = None
        lines.append("Stationary at start: none found; gyro bias left at zero (the filter still estimates it)")

    # ---- ZED odometry
    Z = np.array(zodom) if zodom else np.zeros((0, 6))
    if len(Z) > 1:
        zt, info = choose_time_base(Z[:, 0], Z[:, 1])
        info.update(count=len(Z), rate_hz=round(rate_hz(zt), 2))
        calib["topics"]["zed_odom"] = info
        zyaw = np.unwrap(Z[:, 5])
        with open(out / "zed_odom.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "record_s", "x_m", "y_m", "z_m", "yaw_rad"])
            for i in range(len(Z)):
                wr.writerow([f"{zt[i]:.6f}", f"{Z[i, 0]:.6f}", f"{Z[i, 2]:.4f}", f"{Z[i, 3]:.4f}",
                             f"{Z[i, 4]:.4f}", f"{zyaw[i]:.5f}"])
        lines.append(f"ZED odometry: {len(Z)} messages at {info['rate_hz']} Hz, time base {info['time_base']}")
    else:
        lines.append("ZED odometry: not in bag")

    # ---- LiDAR scan stamps (KISS-ICP poses carry record times; fuse_ekf.py maps them to these)
    Ls = np.array(lidar) if lidar else np.zeros((0, 2))
    if len(Ls) > 1:
        lt, info = choose_time_base(Ls[:, 0], Ls[:, 1])
        info.update(count=len(Ls), rate_hz=round(rate_hz(lt), 2),
                    record_interval_std_s=round(float(np.std(np.diff(Ls[:, 0]))), 4),
                    header_interval_std_s=round(float(np.std(np.diff(Ls[:, 1]))), 4))
        calib["topics"]["lidar"] = info
        with open(out / "lidar_stamps.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "record_s", "header_s"])
            for i in range(len(Ls)):
                wr.writerow([f"{lt[i]:.6f}", f"{Ls[i, 0]:.6f}", f"{Ls[i, 1]:.6f}"])
        lines.append(f"LiDAR: {len(Ls)} scans at {info['rate_hz']} Hz, time base {info['time_base']}, "
                     f"header lag behind record {1000 * info['median_record_minus_header_s']:.0f} ms (median); "
                     f"interval spread {1000 * info['header_interval_std_s']:.1f} ms by header, "
                     f"{1000 * info['record_interval_std_s']:.1f} ms by record time")
    else:
        lines.append("LiDAR: not in bag")

    if cam_info:
        (out / "camera_info.json").write_text(json.dumps(cam_info, indent=2), encoding="utf-8")
        k = cam_info["k"]
        lines.append(f"Camera info: {cam_info['width']} x {cam_info['height']}, fx {k[0]:.1f}, fy {k[4]:.1f}, "
                     f"cx {k[2]:.1f}, cy {k[5]:.1f} ({n_cam_info} messages)")
    else:
        lines.append("Camera info: not in bag")
    if tf_static:
        (out / "tf_static.json").write_text(json.dumps(tf_static, indent=2), encoding="utf-8")
        lines.append(f"Static transforms: {len(tf_static)}")
        for tr in tf_static:
            x, y, z = tr["translation_m"]
            r, p, yw = tr["rpy_deg"]
            lines.append(f"  {tr['parent']} -> {tr['child']}: xyz ({x:.3f}, {y:.3f}, {z:.3f}) m, "
                         f"rpy ({r:.1f}, {p:.1f}, {yw:.1f}) deg")

    (out / "calibration.json").write_text(json.dumps(calib, indent=2), encoding="utf-8")
    (out / "extract_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print()
    print("\n".join(lines))
    print(f"\nResults written to {out.resolve()}")


if __name__ == "__main__":
    main()
