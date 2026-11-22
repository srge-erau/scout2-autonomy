#!/usr/bin/env python3
"""RGB-D visual odometry from the ZED X left image and registered depth.

Runs on a Scout 2.0 rosbag directly (no ROS install needed) and writes, into
results_<bag name>:

    vo.csv            camera trajectory: time, x, y, z, yaw, tracking status per frame
    vo_summary.txt    frames processed, tracking failures, settings
    vo_summary.json   the same, read by evaluate.py

Method: Shi-Tomasi corners are detected on a keyframe, spread over a 4 x 3 grid,
and back-projected to 3D with the registered depth (3 x 3 median, depth edges
rejected). They are tracked into each new image with pyramidal Lucas-Kanade optical
flow with a forward-backward check, and the camera pose relative to the keyframe is
solved with PnP inside RANSAC, then refined on the inliers. A new keyframe is taken
when the inliers thin out or the camera has moved far enough. If tracking against the
keyframe fails, the previous frame is used as a temporary keyframe ("recovered"). If
that also fails the frame is marked "lost", the pose is held, and tracking restarts.

Visual only: no IMU, wheel or LiDAR data enters the pose computation. The result is
expressed in a level frame (x forward, y left, z up) so that yaw is the rotation about
the vertical although the camera is tilted. The vertical is taken from the camera's own
turns: a robot on a level floor rotates about the vertical, so that is the axis of the
camera's rotations. IMU gravity from calibration.json (extract_signals.py) is used only
as a cross-check, or as the fallback when a run has less than 90 deg of turning.

Setup (once):
    pip install rosbags numpy opencv-python

Usage:
    python visual_odometry.py PATH_TO_BAG_FOLDER
    python visual_odometry.py PATH_TO_BAG_FOLDER --stride 1 --limit 300
"""
import argparse
import csv
import json
import math
import sys
import time
import warnings
from pathlib import Path

import numpy as np

R_BC = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)  # optical frame -> body frame (x fwd, y left, z up)


def level_rotation(up):
    """Rotation that turns the measured up direction onto +z by the smallest angle."""
    u = np.asarray(up, float)
    u = u / np.linalg.norm(u)
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(u, z)
    s, c = float(np.linalg.norm(axis)), float(u @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    axis /= s
    ang = math.atan2(s, c)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def up_from_turns(Ts, step_ok, min_step_deg=0.5):
    """The vertical in camera (optical) coordinates, from the camera's own rotations.

    A robot on a level floor turns about the vertical, so the axis of each frame-to-frame
    rotation is the vertical seen from the camera. The rotation vectors of the turning
    steps are stacked and their dominant direction taken (SVD); the sign is chosen so the
    axis points up for an upright camera (optical y points down).
    Returns (unit vector, total rotation in rad, spread) or (None, total, None).
    spread is the second singular value over the first: near 0 when all turns share one axis.
    """
    rv = []
    for k in range(1, len(Ts)):
        if not step_ok[k]:
            continue
        dR = Ts[k - 1][:3, :3].T @ Ts[k][:3, :3]
        r = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        if np.linalg.norm(r) >= math.radians(min_step_deg):
            rv.append(r)
    total = float(sum(np.linalg.norm(r) for r in rv))
    if len(rv) < 10 or total < math.radians(90.0):
        return None, total, None
    _, s, vt = np.linalg.svd(np.array(rv), full_matrices=False)
    u = vt[0]
    if u[1] > 0:
        u = -u
    return u, total, float(s[1] / s[0])


def image_to_gray(msg, msgtype, cv2):
    if msgtype.endswith("CompressedImage"):
        return cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    enc = msg.encoding.lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.asarray(msg.data, dtype=np.uint8)[: h * step].reshape(h, step)
    if enc in ("mono8", "8uc1"):
        return np.ascontiguousarray(buf[:, :w])
    codes = {"bgr8": (3, cv2.COLOR_BGR2GRAY), "rgb8": (3, cv2.COLOR_RGB2GRAY),
             "bgra8": (4, cv2.COLOR_BGRA2GRAY), "rgba8": (4, cv2.COLOR_RGBA2GRAY)}
    if enc not in codes:
        raise ValueError(f"unsupported image encoding {msg.encoding}")
    ch, code = codes[enc]
    return cv2.cvtColor(np.ascontiguousarray(buf[:, : w * ch]).reshape(h, w, ch), code)


def depth_to_meters(msg):
    enc = msg.encoding.lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if enc == "32fc1":
        dt, bpp, scale = np.dtype(">f4" if msg.is_bigendian else "<f4"), 4, 1.0
    elif enc in ("16uc1", "mono16"):
        dt, bpp, scale = np.dtype(">u2" if msg.is_bigendian else "<u2"), 2, 0.001
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding}")
    rows = np.asarray(msg.data, dtype=np.uint8)[: h * step].reshape(h, step)[:, : w * bpp]
    d = np.ascontiguousarray(rows).view(dt).reshape(h, w).astype(np.float32)
    if scale != 1.0:
        d *= scale
        d[d <= 0] = np.nan
    return d


def header_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def rot_angle(R):
    return math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))


def inv_T(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


class VisualOdometry:
    def __init__(self, cv2, K, work_size, args):
        self.cv2 = cv2
        self.K = K
        self.w, self.h = work_size
        self.args = args
        self.lk = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01))
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) if args.clahe else None
        self.kf = None
        self.prev = None
        self.T_w = np.eye(4)
        self.last_strength = None

    # ---- helpers
    def sample_depth(self, depth, px):
        dh, dw = depth.shape
        sx, sy = dw / self.w, dh / self.h
        u = np.clip(np.round((px[:, 0] + 0.5) * sx - 0.5).astype(int), 1, dw - 2)
        v = np.clip(np.round((px[:, 1] + 0.5) * sy - 0.5).astype(int), 1, dh - 2)
        patch = np.stack([depth[v + dv, u + du] for dv in (-1, 0, 1) for du in (-1, 0, 1)], axis=1)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            med = np.nanmedian(patch, axis=1)
            spread = np.nanmax(patch, axis=1) - np.nanmin(patch, axis=1)
        nvalid = np.sum(np.isfinite(patch), axis=1)
        a = self.args
        ok = (nvalid >= 6) & np.isfinite(med) & (med >= a.min_depth) & (med <= a.max_depth) & (spread <= 0.05 * med + 0.02)
        return np.where(ok, med, np.nan)

    def depth_mask(self, depth):
        valid = np.isfinite(depth) & (depth >= self.args.min_depth) & (depth <= self.args.max_depth)
        m = valid.astype(np.uint8) * 255
        if m.shape != (self.h, self.w):
            m = self.cv2.resize(m, (self.w, self.h), interpolation=self.cv2.INTER_NEAREST)
        return m

    def detect(self, gray, mask):
        """Corners spread over a 4 x 3 grid, each above an absolute strength.

        goodFeaturesToTrack's quality level is relative to the strongest corner in view, so
        on a blank or washed-out image it returns sensor noise, which then "tracks" with
        zero motion. Corners are therefore found on a slightly blurred copy (Gaussian,
        sigma 1.5 px), where pixel noise loses most of its gradient but real structure,
        including faint smooth shading, keeps it, and must pass an absolute floor on the
        minimum eigenvalue there. Tracking still runs on the sharp image.
        """
        cv2 = self.cv2
        gx, gy = 4, 3
        per = max(4, self.args.max_features // (gx * gy))
        gray = cv2.GaussianBlur(gray, (0, 0), 1.5)
        strength = cv2.cornerMinEigenVal(gray, 7, ksize=3)
        pts = []
        for j in range(gy):
            for i in range(gx):
                x0, x1 = i * self.w // gx, (i + 1) * self.w // gx
                y0, y1 = j * self.h // gy, (j + 1) * self.h // gy
                m = np.ascontiguousarray(mask[y0:y1, x0:x1])
                if np.count_nonzero(m) < 50:
                    continue
                c = cv2.goodFeaturesToTrack(np.ascontiguousarray(gray[y0:y1, x0:x1]), maxCorners=per,
                                            qualityLevel=0.01, minDistance=self.args.min_distance, mask=m, blockSize=7)
                if c is not None:
                    pts.append(c.reshape(-1, 2) + np.array([x0, y0], np.float32))
        if not pts:
            self.last_strength = None
            return np.zeros((0, 2), np.float32)
        pts = np.concatenate(pts).astype(np.float32)
        s = strength[np.clip(pts[:, 1].round().astype(int), 0, self.h - 1), np.clip(pts[:, 0].round().astype(int), 0, self.w - 1)]
        self.last_strength = (float(np.median(s)), float(np.mean(s >= self.args.min_corner_strength)))
        return pts[s >= self.args.min_corner_strength]

    def backproject(self, px, z):
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        return np.column_stack([(px[:, 0] - cx) / fx * z, (px[:, 1] - cy) / fy * z, z])

    def make_keyframe(self, gray, depth, T_w):
        pts = self.detect(gray, self.depth_mask(depth))
        if len(pts):
            z = self.sample_depth(depth, pts)
            keep = np.isfinite(z)
            pts, z = pts[keep], z[keep]
        P = self.backproject(pts, z) if len(pts) else np.zeros((0, 3))
        return {"P": P, "px": pts.copy(), "T_w": T_w.copy(), "n0": len(pts)}

    def track(self, prev_gray, gray, p0):
        cv2 = self.cv2
        if len(p0) == 0:
            return p0, np.zeros(0, bool)
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0.reshape(-1, 1, 2), None, **self.lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev_gray, p1, None, **self.lk)
        p1, p0r = p1.reshape(-1, 2), p0r.reshape(-1, 2)
        fb = np.linalg.norm(p0 - p0r, axis=1)
        ok = ((st1.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0) & (p1[:, 0] >= 0) & (p1[:, 0] <= self.w - 1)
              & (p1[:, 1] >= 0) & (p1[:, 1] <= self.h - 1))
        return p1, ok

    def pnp(self, P, px):
        """Transform taking points from the reference camera frame into the current one."""
        cv2 = self.cv2
        if len(P) < self.args.min_inliers:
            return None
        P64, px64 = P.astype(np.float64), px.astype(np.float64)
        try:
            ok, rvec, tvec, inl = cv2.solvePnPRansac(P64, px64, self.K, None, iterationsCount=200,
                                                     reprojectionError=self.args.reproj, confidence=0.999,
                                                     flags=cv2.SOLVEPNP_EPNP)
        except cv2.error:
            return None
        if not ok or inl is None or len(inl) < 6:
            return None
        inl = inl.ravel()
        try:
            rvec, tvec = cv2.solvePnPRefineLM(P64[inl], px64[inl], self.K, None, rvec, tvec)
        except cv2.error:
            pass
        proj, _ = cv2.projectPoints(P64, rvec, tvec, self.K, None)
        err = np.linalg.norm(proj.reshape(-1, 2) - px64, axis=1)
        inl = np.flatnonzero(err < self.args.reproj)
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.asarray(tvec).ravel()
        return T, inl

    def plausible(self, T_step, dt):
        dt = max(dt, 1e-3)
        a = self.args
        return (np.linalg.norm(T_step[:3, 3]) <= a.max_speed * dt + 0.05
                and rot_angle(T_step[:3, :3]) <= a.max_rate * dt + 0.05)

    # ---- one frame
    def process(self, t, gray, depth):
        if self.clahe is not None:
            gray = self.clahe.apply(gray)
        info = {"status": "ok", "tracked": 0, "inliers": 0, "keyframe": 0}
        if self.kf is None:
            self.kf = self.make_keyframe(gray, depth, self.T_w)
            info.update(status="init", tracked=len(self.kf["px"]), keyframe=1)
            self.prev = (t, gray, depth)
            return self.T_w.copy(), info
        t_prev, g_prev, d_prev = self.prev
        dt = t - t_prev
        T_prev = self.T_w
        kf = self.kf
        p1, ok = self.track(g_prev, gray, kf["px"])
        kf["P"], kf["px"] = kf["P"][ok], p1[ok]
        info["tracked"] = int(ok.sum())
        good = False
        res = self.pnp(kf["P"], kf["px"])
        if res is not None:
            T_cur_kf, inl = res
            T_cand = kf["T_w"] @ inv_T(T_cur_kf)
            if (len(inl) >= self.args.min_inliers and len(inl) >= 0.3 * len(kf["px"])
                    and self.plausible(inv_T(T_prev) @ T_cand, dt)):
                good = True
                self.T_w = T_cand
                kf["P"], kf["px"] = kf["P"][inl], kf["px"][inl]
                info["inliers"] = int(len(inl))
        if not good:
            tmp = self.make_keyframe(g_prev, d_prev, T_prev)
            q1, ok2 = self.track(g_prev, gray, tmp["px"])
            res = self.pnp(tmp["P"][ok2], q1[ok2])
            if res is not None:
                T_cur_prev, inl = res
                T_cand = T_prev @ inv_T(T_cur_prev)
                if len(inl) >= self.args.min_inliers and self.plausible(inv_T(T_prev) @ T_cand, dt):
                    good = True
                    self.T_w = T_cand
                    info.update(status="recovered", inliers=int(len(inl)))
            if not good:
                info["status"] = "lost"
            self.kf = self.make_keyframe(gray, depth, self.T_w)
            info["keyframe"] = 1
        else:
            T_kf_cur = inv_T(kf["T_w"]) @ self.T_w
            far = (np.linalg.norm(T_kf_cur[:3, 3]) > self.args.kf_distance
                   or rot_angle(T_kf_cur[:3, :3]) > math.radians(self.args.kf_angle))
            thin = info["inliers"] < max(self.args.kf_min_inliers, 0.4 * kf["n0"])
            if far or thin:
                self.kf = self.make_keyframe(gray, depth, self.T_w)
                info["keyframe"] = 1
        self.prev = (t, gray, depth)
        return self.T_w.copy(), info


def main():
    ap = argparse.ArgumentParser(description="RGB-D visual odometry (ZED X left image + registered depth) on a Scout 2.0 rosbag.")
    ap.add_argument("bag", type=Path, help="bag folder, containing the .db3 and metadata.yaml")
    ap.add_argument("--image-topic", default="/zed/zed_node/left/image_rect_color/compressed",
                    help="left image; if absent, the ZED rgb stream (the left camera) is used")
    ap.add_argument("--depth-topic", default="/zed/zed_node/depth/depth_registered")
    ap.add_argument("--info-topic", default="/zed/zed_node/left/camera_info")
    ap.add_argument("--out", type=Path, default=None, help="output folder (default: results_<bag name>)")
    ap.add_argument("--stride", type=int, default=0,
                    help="use every Nth image (default: chosen so about 12 images per second are used)")
    ap.add_argument("--max-width", type=int, default=960, help="downscale wider images to this width, pixels")
    ap.add_argument("--max-features", type=int, default=600)
    ap.add_argument("--min-distance", type=int, default=10, help="minimum spacing between corners, pixels")
    ap.add_argument("--min-corner-strength", type=float, default=5e-5,
                    help="absolute corner strength on the blurred image (OpenCV minimum eigenvalue, 7 px block); rejects sensor noise")
    ap.add_argument("--min-depth", type=float, default=0.3, help="metres")
    ap.add_argument("--max-depth", type=float, default=10.0, help="metres; depth error grows with range squared")
    ap.add_argument("--reproj", type=float, default=2.0, help="RANSAC reprojection threshold, pixels")
    ap.add_argument("--min-inliers", type=int, default=25)
    ap.add_argument("--kf-min-inliers", type=int, default=120)
    ap.add_argument("--kf-distance", type=float, default=1.0, help="new keyframe after this much motion, metres")
    ap.add_argument("--kf-angle", type=float, default=15.0, help="new keyframe after this much rotation, degrees")
    ap.add_argument("--max-speed", type=float, default=2.5, help="reject steps faster than this, m/s")
    ap.add_argument("--max-rate", type=float, default=2.5, help="reject steps turning faster than this, rad/s")
    ap.add_argument("--clahe", action="store_true", help="contrast-equalise images first (dim hallways)")
    ap.add_argument("--no-level", action="store_true", help="do not level the trajectory (keep the camera frame)")
    ap.add_argument("--limit", type=int, default=0, help="stop after this many processed frames (0 = all)")
    args = ap.parse_args()

    try:
        import cv2
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as e:
        sys.exit(f"Missing package: {e.name}. Run: pip install rosbags numpy opencv-python")

    bag = args.bag.expanduser().resolve()
    if not (bag / "metadata.yaml").exists():
        sys.exit(f"{bag} does not look like a ROS 2 bag folder (no metadata.yaml).")
    out = args.out or Path(f"results_{bag.name}")
    out.mkdir(parents=True, exist_ok=True)

    # gravity from the IMU, as a cross-check (and fallback) for the camera's own vertical
    up_imu = None
    if (out / "calibration.json").exists():
        calib = json.loads((out / "calibration.json").read_text(encoding="utf-8"))
        if calib.get("gravity_unit_imu"):
            up_imu = np.asarray(calib["gravity_unit_imu"], float)

    t_start = time.time()
    rows = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        by_topic = {c.topic: c for c in reader.connections}
        rgb = "/zed/zed_node/rgb/image_rect_color/compressed"
        if args.image_topic not in by_topic and rgb in by_topic:
            print(f"Note: {args.image_topic} not in bag; using {rgb} (the ZED rgb stream is the left camera).")
            args.image_topic = rgb
        for needed in (args.image_topic, args.depth_topic):
            if needed not in by_topic:
                sys.exit(f"Topic {needed} not in bag. Available: {sorted(by_topic)}")
        img_conn, dep_conn = by_topic[args.image_topic], by_topic[args.depth_topic]

        info = None
        if args.info_topic in by_topic:
            for conn, _, raw in reader.messages(connections=[by_topic[args.info_topic]]):
                m = reader.deserialize(raw, conn.msgtype)
                info = {"width": int(m.width), "height": int(m.height), "k": [float(v) for v in m.k]}
                break
        if info is None and (out / "camera_info.json").exists():
            info = json.loads((out / "camera_info.json").read_text(encoding="utf-8"))
        if info is None:
            sys.exit("No camera intrinsics: the bag has no camera_info topic and camera_info.json was not found.")

        if args.stride <= 0:
            duration = max((reader.end_time - reader.start_time) / 1e9, 1e-3)
            args.stride = max(1, int(round(img_conn.msgcount / duration / 12.0)))
        n_expected = min(img_conn.msgcount, dep_conn.msgcount) // max(args.stride, 1)
        if args.limit:
            n_expected = min(n_expected, args.limit)
        print(f"Bag: {bag.name}, {img_conn.msgcount} images, {dep_conn.msgcount} depth maps, "
              f"processing about {n_expected} frames (stride {args.stride}).")

        vo = None
        imgs, deps = {}, {}
        n_pairs = 0
        tol = 15_000_000  # ns, image and depth from the same grab
        counts = {"init": 0, "ok": 0, "recovered": 0, "lost": 0}
        sum_inl, n_kf = 0, 0
        setup_note = ""
        first_strength = None
        for conn, stamp, raw in reader.messages(connections=[img_conn, dep_conn]):
            msg = reader.deserialize(raw, conn.msgtype)
            key = header_ns(msg)
            mine, other = (imgs, deps) if conn.topic == args.image_topic else (deps, imgs)
            match = None
            if other:
                near = min(other, key=lambda k: abs(k - key))
                if abs(near - key) <= tol:
                    match = near
            if match is None:
                mine[key] = (stamp, msg)
                if n_pairs == 0 and len(imgs) + len(deps) >= 40:
                    ki, kd = max(imgs, default=None), max(deps, default=None)
                    gap = f"{abs(ki - kd) / 1e6:.1f} ms" if ki is not None and kd is not None else "n/a"
                    sys.exit(f"Images and depth maps never share a timestamp (latest ones {gap} apart, tolerance "
                             f"{tol / 1e6:.0f} ms). Check --image-topic and --depth-topic.")
                for d in (imgs, deps):
                    if len(d) > 60:
                        for k in sorted(d)[:-30]:
                            del d[k]
                continue
            o_stamp, o_msg = other.pop(match)
            for d in (imgs, deps):
                for k in [k for k in d if k < min(key, match)]:
                    del d[k]
            n_pairs += 1
            if (n_pairs - 1) % max(args.stride, 1):
                continue
            if conn.topic == args.image_topic:
                img_msg, img_type, dep_msg, rec = msg, conn.msgtype, o_msg, stamp
                head = key
            else:
                img_msg, img_type, dep_msg, rec = o_msg, img_conn.msgtype, msg, o_stamp
                head = match
            try:
                gray = image_to_gray(img_msg, img_type, cv2)
                depth = depth_to_meters(dep_msg)
                if gray is None:
                    raise ValueError("image could not be decoded")
            except ValueError as e:
                print(f"  skipping frame: {e}")
                continue
            if vo is None:
                h0, w0 = gray.shape
                s = min(1.0, args.max_width / w0)
                work = (int(round(w0 * s)), int(round(h0 * s)))
                sx_i, sy_i = work[0] / info["width"], work[1] / info["height"]
                k = info["k"]
                K = np.array([[k[0] * sx_i, 0, (k[2] + 0.5) * sx_i - 0.5],
                              [0, k[4] * sy_i, (k[5] + 0.5) * sy_i - 0.5],
                              [0, 0, 1.0]])
                vo = VisualOdometry(cv2, K, work, args)
                setup_note = (f"image {w0} x {h0} -> working {work[0]} x {work[1]}, depth {dep_msg.width} x "
                              f"{dep_msg.height} {dep_msg.encoding}, camera_info {info['width']} x {info['height']}")
                print(f"First frame: {setup_note}")
            if gray.shape[1] != vo.w:
                gray = cv2.resize(gray, (vo.w, vo.h), interpolation=cv2.INTER_AREA)
            T_w, st = vo.process(head / 1e9, gray, depth)
            if st["status"] == "init":
                ls = vo.last_strength
                first_strength = ls
                extra = (f" (median corner strength {ls[0]:.1e}, {100 * ls[1]:.0f}% above the "
                         f"{args.min_corner_strength:.0e} floor)") if ls else ""
                print(f"  {st['tracked']} corners with valid depth on the first keyframe{extra}")
            counts[st["status"]] += 1
            sum_inl += st["inliers"]
            n_kf += st["keyframe"]
            rows.append((head / 1e9, rec / 1e9, T_w, st["status"], st["tracked"], st["inliers"], st["keyframe"]))
            n = len(rows)
            if n % 200 == 0:
                rate = n / (time.time() - t_start)
                left = (n_expected - n) / rate if rate else 0
                okf = (counts["ok"] + counts["recovered"]) / max(1, n - counts["init"])
                print(f"  {n}/{n_expected} frames, {100 * okf:.0f}% tracked, {counts['lost']} lost, "
                      f"{sum_inl / max(1, n):.0f} inliers per frame, {rate:.1f} frames/s, about {left / 60:.1f} min left")
            if args.limit and n >= args.limit:
                break

    if len(rows) < 2:
        sys.exit("Fewer than two frames processed; nothing to write.")

    # ---- level frame: vertical from the camera's own turns, IMU gravity as cross-check or fallback
    Ts = [r[2] for r in rows]
    step_ok = np.array([r[3] in ("ok", "recovered") for r in rows])
    up_vo, turned, spread = up_from_turns(Ts, step_ok)
    up_imu_opt = R_BC.T @ (up_imu / np.linalg.norm(up_imu)) if up_imu is not None else None
    agree = (math.degrees(math.acos(max(-1.0, min(1.0, float(up_vo @ up_imu_opt)))))
             if up_vo is not None and up_imu_opt is not None else None)
    if args.no_level:
        up_opt, level_note = None, "not levelled (--no-level)"
    elif up_vo is not None and spread < 0.3:
        up_opt = up_vo
        level_note = f"levelled with the vertical from the camera's own turns ({math.degrees(turned):.0f} deg of turning)"
        if agree is not None:
            level_note += f"; IMU gravity agrees within {agree:.1f} deg"
    elif up_imu_opt is not None:
        up_opt = up_imu_opt
        level_note = "levelled with IMU gravity from calibration.json (too little turning to use the camera's own axis)"
    else:
        up_opt, level_note = None, "not levelled (too little turning and no calibration.json)"
    if agree is not None and agree > 5.0:
        print(f"WARNING: the camera's turning axis and IMU gravity differ by {agree:.1f} deg. The IMU frame may "
              f"be rotated relative to the camera frame; the camera's own axis was used.")
    R_LB = level_rotation(R_BC @ up_opt) if up_opt is not None else np.eye(3)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float((R_BC @ up_opt)[2]))))) if up_opt is not None else None
    if tilt is not None:
        level_note += f"; camera tilt {tilt:.1f} deg"
    A = np.eye(4)
    A[:3, :3] = R_LB @ R_BC
    A_inv = inv_T(A)
    poses = []
    for T_w in Ts:
        T_L = A @ T_w @ A_inv
        poses.append((T_L[0, 3], T_L[1, 3], T_L[2, 3], math.atan2(T_L[1, 0], T_L[0, 0])))
    poses = np.array(poses)

    head_t = np.array([r[0] for r in rows])
    rec_t = np.array([r[1] for r in rows])
    lat = rec_t - head_t
    use_header = abs(float(np.median(lat))) < 1.0 and np.all(np.diff(head_t) > 0)
    t_out = head_t if use_header else rec_t
    yaw_u = np.unwrap(poses[:, 3])
    with open(out / "vo.csv", "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "record_s", "x_m", "y_m", "z_m", "yaw_rad", "status", "tracked", "inliers", "keyframe"])
        for r, p, tt, yw in zip(rows, poses, t_out, yaw_u):
            wr.writerow([f"{tt:.6f}", f"{r[1]:.6f}", f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{yw:.5f}",
                         r[3], r[4], r[5], r[6]])

    n = len(rows)
    xy = poses[:, :2]
    runtime = time.time() - t_start
    summary = {
        "bag": bag.name, "frames": n, "stride": args.stride,
        "duration_s": round(float(t_out[-1] - t_out[0]), 2),
        "status_counts": counts,
        "tracked_fraction": round((counts["ok"] + counts["recovered"]) / max(1, n - counts["init"]), 4),
        "lost_fraction": round(counts["lost"] / max(1, n - counts["init"]), 4),
        "keyframes": n_kf, "mean_inliers": round(sum_inl / n, 1),
        "time_base": "header" if use_header else "record",
        "first_keyframe_corner_strength_median": first_strength[0] if first_strength else None,
        "setup": setup_note, "levelling": level_note,
        "vertical_from_turns_vs_imu_deg": round(agree, 2) if agree is not None else None,
        "final_xy_m": [round(float(xy[-1, 0]), 3), round(float(xy[-1, 1]), 3)],
        "final_z_m": round(float(poses[-1, 2]), 3),
        "runtime_s": round(runtime, 1),
        "settings": {k: v for k, v in vars(args).items() if k not in ("bag", "out")},
    }
    (out / "vo_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    lines = [
        f"Bag: {bag.name}",
        f"Frames processed: {n} (every {args.stride} image), {summary['duration_s']:.1f} s",
        f"Setup: {setup_note}",
        f"Levelling: {level_note}",
        f"Tracking: {counts['ok']} ok, {counts['recovered']} recovered from the previous frame, "
        f"{counts['lost']} lost ({100 * summary['lost_fraction']:.1f}%)",
        f"Keyframes: {n_kf}, mean PnP inliers per frame: {summary['mean_inliers']:.0f}",
        f"End point relative to start: x {xy[-1, 0]:.2f} m, y {xy[-1, 1]:.2f} m, z {poses[-1, 2]:.2f} m, "
        f"distance {np.linalg.norm(xy[-1]):.2f} m",
        f"Runtime: {runtime / 60:.1f} min",
    ]
    (out / "vo_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print()
    print("\n".join(lines))
    print(f"\nResults written to {out.resolve()}")


if __name__ == "__main__":
    main()
