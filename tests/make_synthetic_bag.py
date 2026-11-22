#!/usr/bin/env python3
"""Synthetic Scout 2.0 rosbag2 with known ground truth, for testing the odometry benchmark.

A robot drives 8 m down a textured corridor, turns 180 deg in place, drives back,
turns 180 deg again and stops on its start mark (55 s). The bag carries the topics of
hallway_04_cmd with known errors:

    /agilex_odom          wheel odometry, +6.6 % distance and +35 % yaw rate (skid-steer slip)
    /zed/zed_node/imu/data  gyro bias (0.004, -0.003, 0.005) rad/s, camera tilted 8 deg down
    /zed/zed_node/odom    ZED-style odometry, 2 % short, yaw drift 0.003 rad/s
    left image + depth    ray-cast corridor, JPEG images, depth noise growing with range squared
    /rslidar_points       tiny clouds; header stamps exact, record times 80 ms late with
                          jitter and late-then-burst arrivals
    /tf_static            base_link -> zed_camera_link and rslidar

It also writes what kiss_vs_wheel.py would have produced (kiss_icp_poses.csv: 3 % short,
3 deg alignment error, two scans missing; wheel_odom.csv) and truth.csv.

Usage (writes the bag folder and a results folder next to it):
    python make_synthetic_bag.py bags/synth_loop --csv-dir results_synth_loop
    python make_synthetic_bag.py bags/synth_dark --csv-dir results_synth_dark --blackout 12 14
Options: --depth-encoding 16UC1, --depth-scale 1.0, --zero-twist, --rects 120 (blank walls),
--gyro-scale 0.987 (gyro reads 1.3 % low).
"""
import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS2_HUMBLE)
M = TS.types
Time = M["builtin_interfaces/msg/Time"]
Header = M["std_msgs/msg/Header"]
Point = M["geometry_msgs/msg/Point"]
Quat = M["geometry_msgs/msg/Quaternion"]
Vec3 = M["geometry_msgs/msg/Vector3"]
Pose = M["geometry_msgs/msg/Pose"]
PoseCov = M["geometry_msgs/msg/PoseWithCovariance"]
Twist = M["geometry_msgs/msg/Twist"]
TwistCov = M["geometry_msgs/msg/TwistWithCovariance"]
Odometry = M["nav_msgs/msg/Odometry"]
Imu = M["sensor_msgs/msg/Imu"]
CompressedImage = M["sensor_msgs/msg/CompressedImage"]
Image = M["sensor_msgs/msg/Image"]
CameraInfo = M["sensor_msgs/msg/CameraInfo"]
ROI = M["sensor_msgs/msg/RegionOfInterest"]
TFMessage = M["tf2_msgs/msg/TFMessage"]
TransformStamped = M["geometry_msgs/msg/TransformStamped"]
Transform = M["geometry_msgs/msg/Transform"]

T0 = 1_800_000_000.0          # epoch seconds at bag start
DT = 0.001                    # truth integration step
W_IMG, H_IMG = 320, 240
FX = FY = 260.0
CX, CY = 160.0, 120.0
CAM_OFFSET = np.array([0.25, 0.0, 0.5])   # camera position in base frame
CAM_PITCH = math.radians(8.0)              # nose down
BOX = dict(xmin=-3.0, xmax=13.0, ymin=-1.2, ymax=1.2, zmin=0.0, zmax=2.6)
TEX_PPM = 120.0


def stamp(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def header(t, frame):
    return Header(stamp=stamp(t), frame_id=frame)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def quat_from_matrix(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return Quat(x=x, y=y, z=z, w=w)


R_BC = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)  # optical -> body


def ramp_profile(t, total, ramp, peak):
    """Cosine ramp up, hold, cosine ramp down."""
    if t < 0 or t > total:
        return 0.0
    if t < ramp:
        return peak * 0.5 * (1 - math.cos(math.pi * t / ramp))
    if t > total - ramp:
        return peak * 0.5 * (1 - math.cos(math.pi * (total - t) / ramp))
    return peak


def build_truth():
    phases = []  # (kind, duration, ramp, peak)
    phases.append(("still", 6.0, 0, 0))
    V, W, ramp = 0.6, 0.5, 1.0
    phases.append(("drive", 8.0 / V + ramp, ramp, V))
    phases.append(("still", 1.0, 0, 0))
    phases.append(("turn", math.pi / W + ramp, ramp, W))
    phases.append(("still", 1.0, 0, 0))
    phases.append(("drive", 8.0 / V + ramp, ramp, V))
    phases.append(("still", 1.0, 0, 0))
    phases.append(("turn", math.pi / W + ramp, ramp, W))
    phases.append(("still", 3.0, 0, 0))
    total = sum(p[1] for p in phases)
    n = int(total / DT) + 1
    t = np.arange(n) * DT
    v = np.zeros(n)
    w = np.zeros(n)
    start = 0.0
    windows = []
    for kind, dur, rp, peak in phases:
        idx = (t >= start) & (t < start + dur)
        if kind == "drive":
            v[idx] = [ramp_profile(tt - start, dur, rp, peak) for tt in t[idx]]
            windows.append((start + rp, start + dur - rp))
        elif kind == "turn":
            w[idx] = [ramp_profile(tt - start, dur, rp, peak) for tt in t[idx]]
        start += dur
    th = np.concatenate([[0.0], np.cumsum(w[:-1] * DT)])
    x = np.concatenate([[0.0], np.cumsum(v[:-1] * np.cos(th[:-1]) * DT)])
    y = np.concatenate([[0.0], np.cumsum(v[:-1] * np.sin(th[:-1]) * DT)])
    return t, x, y, th, v, w, windows


def make_texture(seed=7, n_rect=9000):
    rng = np.random.default_rng(seed)
    size = 2048
    tex = np.full((size, size), 128, np.uint8)
    noise = cv2.resize(rng.integers(90, 170, (64, 64)).astype(np.uint8), (size, size), interpolation=cv2.INTER_CUBIC)
    tex = cv2.addWeighted(tex, 0.5, noise, 0.5, 0)
    for _ in range(n_rect):
        w, h = rng.integers(4, 40, 2)
        x0, y0 = rng.integers(0, size - 40, 2)
        tex[y0:y0 + h, x0:x0 + w] = int(rng.integers(25, 230))
    tex = cv2.GaussianBlur(tex, (0, 0), 0.8)
    pyr = [tex.astype(np.float32)]
    for _ in range(4):
        pyr.append(cv2.pyrDown(pyr[-1]))
    return pyr


U, V_ = np.meshgrid(np.arange(W_IMG) + 0.5, np.arange(H_IMG) + 0.5)
DC = np.stack([(U - CX) / FX, (V_ - CY) / FY, np.ones_like(U)], -1).reshape(-1, 3)
PLANES = [(1, BOX["ymin"], 0), (1, BOX["ymax"], 1), (2, BOX["zmin"], 2), (2, BOX["zmax"], 3),
          (0, BOX["xmin"], 4), (0, BOX["xmax"], 5)]
OFF_A = np.array([0.0, 3.1, 5.3, 1.7, 7.9, 2.3])
OFF_B = np.array([0.0, 4.4, 2.2, 6.1, 1.1, 8.8])
SHADE = np.array([1.0, 0.95, 0.8, 1.1, 0.9, 0.85])


def render(R_wc, c_w, pyr):
    dw = DC @ R_wc.T
    n = dw.shape[0]
    tbest = np.full(n, np.inf)
    pid = np.full(n, -1)
    for axis, value, p in PLANES:
        with np.errstate(divide="ignore", invalid="ignore"):
            tt = (value - c_w[axis]) / dw[:, axis]
        ok = (tt > 1e-6) & (tt < tbest)
        tbest[ok] = tt[ok]
        pid[ok] = p
    P = c_w + dw * tbest[:, None]
    a = np.where(pid <= 3, P[:, 0], P[:, 1])
    b = np.where(pid <= 1, P[:, 2], np.where(pid <= 3, P[:, 1], P[:, 2]))
    a = a + OFF_A[pid]
    b = b + OFF_B[pid]
    level = np.clip(np.floor(np.log2(np.maximum(tbest * TEX_PPM / FX, 1.0))), 0, 4).astype(int)
    img = np.zeros(n, np.float32)
    for L in range(5):
        m = level == L
        if not m.any():
            continue
        tex = pyr[L]
        s = TEX_PPM / (2 ** L)
        ia = np.floor(a[m] * s).astype(np.int64) % tex.shape[1]
        ib = np.floor(b[m] * s).astype(np.int64) % tex.shape[0]
        img[m] = tex[ib, ia]
    img *= SHADE[pid]
    depth = tbest.astype(np.float32)
    return img.reshape(H_IMG, W_IMG), depth.reshape(H_IMG, W_IMG)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path, help="bag folder to create (must not exist)")
    ap.add_argument("--csv-dir", type=Path, required=True, help="where to write kiss/wheel CSVs and truth")
    ap.add_argument("--image-rate", type=float, default=20.0)
    ap.add_argument("--depth-scale", type=float, default=0.5, help="depth resolution relative to image")
    ap.add_argument("--depth-encoding", default="32FC1", choices=["32FC1", "16UC1"])
    ap.add_argument("--rects", type=int, default=9000, help="texture density (fewer = blanker walls)")
    ap.add_argument("--blackout", type=float, nargs=2, default=None, help="blank images between these times (s)")
    ap.add_argument("--gyro-scale", type=float, default=1.0, help="gyro scale factor (0.987 = reads 1.3 %% low)")
    ap.add_argument("--image-topic", default="/zed/zed_node/left/image_rect_color/compressed")
    ap.add_argument("--zero-twist", action="store_true", help="leave wheel twist empty (tests the fallback)")
    args = ap.parse_args()
    rng = np.random.default_rng(1)

    t, x, y, th, v, w, windows = build_truth()
    n = len(t)
    print(f"truth: {t[-1]:.1f} s, drive windows {[(round(a, 1), round(b, 1)) for a, b in windows]}")

    # camera (= IMU) body frame pose in world
    cam_p = np.column_stack([x + CAM_OFFSET[0] * np.cos(th), y + CAM_OFFSET[0] * np.sin(th),
                             np.full(n, CAM_OFFSET[2])])
    R_pitch = rot_y(CAM_PITCH)

    msgs = []  # (record_time, topic, msgtype, msg)

    # ---- tf_static
    tfs = [TransformStamped(header=header(T0, "base_link"), child_frame_id="zed_camera_link",
                            transform=Transform(translation=Vec3(x=0.25, y=0.0, z=0.5), rotation=quat_from_matrix(R_pitch))),
           TransformStamped(header=header(T0, "base_link"), child_frame_id="rslidar",
                            transform=Transform(translation=Vec3(x=0.0, y=0.0, z=0.6), rotation=Quat(x=0.0, y=0.0, z=0.0, w=1.0)))]
    msgs.append((T0 + 0.01, "/tf_static", "tf2_msgs/msg/TFMessage", TFMessage(transforms=tfs)))

    # ---- IMU 200 Hz
    bias = np.array([0.004, -0.003, 0.005])
    acc_w = np.gradient(np.gradient(cam_p, DT, axis=0), DT, axis=0)
    step = int(round(0.005 / DT))
    for i in range(0, n, step):
        Rwb = rot_z(th[i]) @ R_pitch
        omega_b = args.gyro_scale * (Rwb.T @ np.array([0, 0, w[i]])) + bias + rng.normal(0, 0.003, 3)
        f_b = Rwb.T @ (acc_w[i] + np.array([0, 0, 9.81])) + rng.normal(0, 0.05, 3)
        q = quat_from_matrix(Rwb)
        msg = Imu(header=header(T0 + t[i], "zed_imu_link"), orientation=q, orientation_covariance=np.zeros(9),
                  angular_velocity=Vec3(x=omega_b[0], y=omega_b[1], z=omega_b[2]), angular_velocity_covariance=np.zeros(9),
                  linear_acceleration=Vec3(x=f_b[0], y=f_b[1], z=f_b[2]), linear_acceleration_covariance=np.zeros(9))
        msgs.append((T0 + t[i] + 0.001 + abs(rng.normal(0, 0.0005)), "/zed/zed_node/imu/data", "sensor_msgs/msg/Imu", msg))

    # ---- wheel odometry 50 Hz, skid-steer errors: +6.6 % distance, +35 % rotation
    step = int(round(0.02 / DT))
    xw = yw = thw = 0.0
    wheel_rows = []
    last_t = None
    for i in range(0, n, step):
        vw = 1.066 * v[i] + rng.normal(0, 0.004)
        ww = 1.35 * w[i] + rng.normal(0, 0.01)
        if abs(v[i]) < 1e-6 and abs(w[i]) < 1e-6:
            vw, ww = 0.0, 0.0
        if last_t is not None:
            dtt = t[i] - last_t
            xw += vw * math.cos(thw) * dtt
            yw += vw * math.sin(thw) * dtt
            thw += ww * dtt
        last_t = t[i]
        q = quat_from_matrix(rot_z(thw))
        tw = Twist(linear=Vec3(x=0.0 if args.zero_twist else vw, y=0.0, z=0.0),
                   angular=Vec3(x=0.0, y=0.0, z=0.0 if args.zero_twist else ww))
        msg = Odometry(header=header(T0 + t[i], "odom"), child_frame_id="base_link",
                       pose=PoseCov(pose=Pose(position=Point(x=xw, y=yw, z=0.0), orientation=q), covariance=np.zeros(36)),
                       twist=TwistCov(twist=tw, covariance=np.zeros(36)))
        rec = T0 + t[i] + 0.002 + abs(rng.normal(0, 0.001))
        msgs.append((rec, "/agilex_odom", "nav_msgs/msg/Odometry", msg))
        wheel_rows.append((rec, xw, yw, thw))

    # ---- ZED odometry at image rate: camera pose with yaw drift and 2 % short scale
    step_img = int(round(1.0 / args.image_rate / DT))
    dcam = np.diff(cam_p, axis=0, prepend=cam_p[:1])
    drift = 0.003 * t
    zx = np.cumsum(0.98 * (np.cos(drift) * dcam[:, 0] - np.sin(drift) * dcam[:, 1]))
    zy = np.cumsum(0.98 * (np.sin(drift) * dcam[:, 0] + np.cos(drift) * dcam[:, 1]))
    zx -= zx[0]
    zy -= zy[0]
    zth = th + drift
    for i in range(0, n, step_img):
        Rz = rot_z(zth[i]) @ R_pitch
        msg = Odometry(header=header(T0 + t[i], "odom"), child_frame_id="zed_camera_link",
                       pose=PoseCov(pose=Pose(position=Point(x=zx[i], y=zy[i], z=0.0), orientation=quat_from_matrix(Rz)),
                                    covariance=np.zeros(36)),
                       twist=TwistCov(twist=Twist(linear=Vec3(x=0.0, y=0.0, z=0.0), angular=Vec3(x=0.0, y=0.0, z=0.0)),
                                      covariance=np.zeros(36)))
        msgs.append((T0 + t[i] + 0.05, "/zed/zed_node/odom", "nav_msgs/msg/Odometry", msg))

    # ---- camera_info, images, depth
    pyr = make_texture(n_rect=args.rects)
    K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    P = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
    dW, dH = int(W_IMG * args.depth_scale), int(H_IMG * args.depth_scale)
    n_frames = 0
    for i in range(0, n, step_img):
        ti = T0 + t[i]
        R_wb = rot_z(th[i]) @ R_pitch
        R_wc = R_wb @ R_BC
        img, depth = render(R_wc, cam_p[i], pyr)
        if args.blackout and args.blackout[0] <= t[i] <= args.blackout[1]:
            img = np.full_like(img, 90.0)
        img = np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8)
        ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        msgs.append((ti + 0.04, args.image_topic, "sensor_msgs/msg/CompressedImage",
                     CompressedImage(header=header(ti, "zed_left_camera_optical_frame"), format="bgr8; jpeg compressed bgr8",
                                     data=np.frombuffer(jpg.tobytes(), np.uint8))))
        dn = depth + rng.normal(0, 1, depth.shape).astype(np.float32) * (0.002 * depth ** 2 + 0.003)
        dn[rng.random(depth.shape) < 0.02] = np.nan
        if args.depth_scale != 1.0:
            dn = cv2.resize(dn, (dW, dH), interpolation=cv2.INTER_NEAREST)
        if args.depth_encoding == "16UC1":
            dd = np.nan_to_num(dn * 1000.0, nan=0.0).clip(0, 65535).astype(np.uint16)
            data, step_b = dd.tobytes(), dW * 2
        else:
            data, step_b = dn.astype(np.float32).tobytes(), dW * 4
        msgs.append((ti + 0.045, "/zed/zed_node/depth/depth_registered", "sensor_msgs/msg/Image",
                     Image(header=header(ti, "zed_left_camera_optical_frame"), height=dH, width=dW,
                           encoding=args.depth_encoding, is_bigendian=0, step=step_b, data=np.frombuffer(data, np.uint8))))
        msgs.append((ti + 0.041, "/zed/zed_node/left/camera_info", "sensor_msgs/msg/CameraInfo",
                     CameraInfo(header=header(ti, "zed_left_camera_optical_frame"), height=H_IMG, width=W_IMG,
                                distortion_model="plumb_bob", d=np.zeros(5), k=np.array(K), r=np.eye(3).ravel(),
                                p=np.array(P), binning_x=0, binning_y=0,
                                roi=ROI(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))))
        n_frames += 1
    print(f"rendered {n_frames} frames")

    # ---- LiDAR scans (tiny clouds; only the stamps matter): record time 80 ms late, jitter,
    # 3 % of scans delayed 60-110 ms, recorder order preserved (late, then a burst)
    PointCloud2 = M["sensor_msgs/msg/PointCloud2"]
    PointField = M["sensor_msgs/msg/PointField"]
    fields = [PointField(name=nm, offset=4 * j, datatype=7, count=1) for j, nm in enumerate("xyz")]
    lidar_rec = []
    last = -1.0
    for i in range(0, n, int(round(0.1 / DT))):
        ts_scan = T0 + t[i]
        rec = ts_scan + 0.08 + rng.normal(0, 0.003)
        if rng.random() < 0.03:
            rec += rng.uniform(0.06, 0.11)
        rec = max(rec, last + 0.0001)
        last = rec
        lidar_rec.append(rec)
        pts = rng.uniform(-5, 5, (50, 3)).astype(np.float32)
        msgs.append((rec, "/rslidar_points", "sensor_msgs/msg/PointCloud2",
                     PointCloud2(header=header(ts_scan, "rslidar"), height=1, width=50, fields=fields,
                                 is_bigendian=False, point_step=12, row_step=600,
                                 data=np.frombuffer(pts.tobytes(), np.uint8), is_dense=True)))

    # ---- write the bag in record-time order
    msgs.sort(key=lambda m: m[0])
    with Writer(args.out, version=8) as writer:
        conns = {}
        for rec, topic, mtype, msg in msgs:
            if topic not in conns:
                conns[topic] = writer.add_connection(topic, mtype, typestore=TS)
            writer.write(conns[topic], int(rec * 1e9), TS.serialize_cdr(msg, mtype))
    print(f"wrote {len(msgs)} messages to {args.out}")

    # ---- CSVs kiss_vs_wheel.py would have written
    args.csv_dir.mkdir(parents=True, exist_ok=True)
    with open(args.csv_dir / "wheel_odom.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "x_m", "y_m", "yaw_rad"])
        for rec, a, b, c in wheel_rows:
            wr.writerow([f"{rec:.6f}", f"{a:.4f}", f"{b:.4f}", f"{c:.5f}"])
    # KISS-ICP: 10 Hz, 3 % short along the corridor, 1 cm / 0.1 deg jitter, alignment off by
    # 3 deg (as when aligned against drifting wheel odometry). Rows carry bag record times:
    # 80 ms after the scan, with jitter and occasional late-then-burst arrivals; two scans missing.
    step = int(round(0.1 / DT))
    dx = np.diff(x, prepend=x[0]) * 0.97
    dy = np.diff(y, prepend=y[0])
    kx, ky = np.cumsum(dx), np.cumsum(dy)
    rot = math.radians(3.0)
    drop = {150, 333}
    with open(args.csv_dir / "kiss_icp_poses.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "x_m", "y_m", "z_m", "yaw_rad"])
        for k, (i, rec) in enumerate(zip(range(0, n, step), lidar_rec)):
            if k in drop:
                continue
            px = kx[i] + rng.normal(0, 0.01)
            py = ky[i] + rng.normal(0, 0.01)
            qx = math.cos(rot) * px - math.sin(rot) * py
            qy = math.sin(rot) * px + math.cos(rot) * py
            yaw = th[i] + 0.0005 * t[i] + rng.normal(0, 0.0017)
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            wr.writerow([f"{rec:.6f}", f"{qx:.4f}", f"{qy:.4f}", "0.0000", f"{yaw:.5f}"])
    with open(args.csv_dir / "truth.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "x_m", "y_m", "yaw_rad"])
        for i in range(0, n, 10):
            wr.writerow([f"{T0 + t[i]:.6f}", f"{x[i]:.4f}", f"{y[i]:.4f}", f"{th[i]:.5f}"])
    (args.csv_dir / "truth_windows.txt").write_text(
        " ".join(f"{a:.1f} {b:.1f}" for a, b in windows) + "\n")


if __name__ == "__main__":
    main()
