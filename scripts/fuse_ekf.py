#!/usr/bin/env python3
"""Fuse the Scout 2.0 odometry sources with one planar EKF, once per sensor set.

Reads the CSVs written by kiss_vs_wheel.py, extract_signals.py and visual_odometry.py
from results_<bag name> and writes, for every sensor set it has data for:

    traj_<config>.csv     fused trajectory: time, x, y, yaw, speed, yaw rate, gyro bias
    gating_<config>.csv   gated measurements: time, sensor, NIS, accepted
    ekf_summary.json      per configuration and sensor: measurements, rejections, noise,
                          time offset, mounting correction (read by evaluate.py)
    ekf_summary.txt       the same, readable

Sensor sets (configurations):
    wheel                  wheel speed and yaw rate
    wheel_inertial         wheel + ZED IMU gyro
    visual                 visual odometry (visual_odometry.py)
    visual_inertial        visual + gyro
    lidar                  KISS-ICP LiDAR odometry
    lidar_inertial         LiDAR + gyro
    visual_lidar_inertial  visual + LiDAR + gyro
    all                    wheel + visual + LiDAR + gyro

Every configuration uses the same filter, motion model and noise settings; they differ
only in which sensors are switched on. The fusion is loosely coupled.

State: x, y, heading, forward speed v, yaw rate w, gyro bias b, gyro scale error s (fixed
at zero unless --estimate-scale), and one heading clone per LiDAR or visual source.
Motion model: constant speed and yaw rate, with white acceleration noise. No lateral
speed (skid-steer, non-holonomic), so the track is that of the robot's turning centre.

Measurements:
    wheel:          speed and yaw rate, 0.2 s means, 2 degrees of freedom
    gyro:           (1 + s) w + b at full rate, 1 degree of freedom
    LiDAR, visual:  speed over each 0.2 s interval, and the heading change over the same
                    interval against the heading cloned at its start (1 degree of freedom
                    each). A heading change constrains the heading itself, so the
                    sensor's long-term heading carries through even when the gyro
                    dominates the short-term yaw rate.

Each measurement passes a chi-square innovation gate (99 %) before it is used. The
rejections per sensor are the consistency check between dissimilar sensors.

Calibration done here, reported in ekf_summary: the time offset of each source relative
to the gyro (cross-correlation of yaw rates), and for LiDAR and visual odometry the
small yaw between the sensor's frame and the direction of travel (from straight
segments, where a skid-steer robot has no sideways speed).

Usage:
    python fuse_ekf.py PATH_TO_BAG_FOLDER        (or the results_<bag name> folder)
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

CONFIGS = {
    "wheel": ("wheel",),
    "wheel_inertial": ("wheel", "gyro"),
    "visual": ("visual",),
    "visual_inertial": ("visual", "gyro"),
    "lidar": ("lidar",),
    "lidar_inertial": ("lidar", "gyro"),
    "visual_lidar_inertial": ("visual", "lidar", "gyro"),
    "all": ("wheel", "visual", "lidar", "gyro"),
}
WINDOW_S = 0.2           # averaging window for velocity sources
CHI2_99 = {1: 6.635, 2: 9.210}
SIGMA_ACC = 1.0          # m/s^2, white acceleration noise on v
SIGMA_ALPHA = 2.0        # rad/s^2, white angular acceleration noise on w
SIGMA_BIAS_RW = 2e-4     # rad/s per sqrt(s), gyro bias random walk
SIGMA_SCALE0 = 0.02      # initial uncertainty of the gyro scale error (2 %)
SIGMA_SCALE_RW = 1e-5    # per sqrt(s), gyro scale random walk
HEADING_FRAC = {"lidar": 0.01, "visual": 0.02}   # heading-change error proportional to the turn
HEADING_WINDOW_S = 0.2   # interval for LiDAR and visual heading-change measurements


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    head, body = rows[0], [r for r in rows[1:] if r]
    cols = {}
    for j, name in enumerate(head):
        vals = [r[j] for r in body]
        try:
            cols[name] = np.array(vals, dtype=float)
        except ValueError:
            cols[name] = np.array(vals)
    return cols


def resolve_results(arg):
    p = arg.expanduser().resolve()
    if (p / "metadata.yaml").exists():
        return Path(f"results_{p.name}").resolve(), p.name
    if p.name.startswith("results_"):
        return p, p.name[len("results_"):]
    return p, p.name


def velocity_from_pose(t, x, y, yaw, valid_step=None, window=WINDOW_S):
    """Body-frame velocity (forward, sideways) and yaw rate averaged over about `window` seconds.

    Returns rows (t_mid, vx, vy, w, dt, t_start, t_end). Intervals that contain an invalid
    step are skipped.
    """
    n = len(t)
    if n < 3:
        return np.zeros((0, 7))
    dt_med = float(np.median(np.diff(t)))
    k = max(1, int(round(window / max(dt_med, 1e-3))))
    idx = np.arange(0, n, k)
    if valid_step is None:
        valid_step = np.ones(n, bool)
    bad = np.concatenate([[0], np.cumsum(~valid_step)])  # bad[i] = invalid steps among 0..i-1
    rows = []
    for a, b in zip(idx[:-1], idx[1:]):
        if bad[b + 1] - bad[a + 1] > 0:
            continue
        dt = t[b] - t[a]
        if dt <= 0 or dt > 5 * window:
            continue
        dth = yaw[b] - yaw[a]
        mid = yaw[a] + 0.5 * dth
        dx, dy = x[b] - x[a], y[b] - y[a]
        c, s = math.cos(mid), math.sin(mid)
        rows.append((0.5 * (t[a] + t[b]), (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dth / dt, dt, t[a], t[b]))
    return np.array(rows) if rows else np.zeros((0, 7))


def average_rates(t, v, w, window=WINDOW_S):
    """Average directly measured speed and yaw rate over consecutive windows."""
    if len(t) < 2:
        return np.zeros((0, 5))
    edges = np.arange(t[0], t[-1] + window, window)
    which = np.digitize(t, edges) - 1
    rows = []
    for k in np.unique(which):
        m = which == k
        if m.sum() < 2:
            continue
        rows.append((float(np.mean(t[m])), float(np.mean(v[m])), 0.0, float(np.mean(w[m])), window))
    return np.array(rows)


def gyro_usable(rec, head, g, frozen_run=50):
    """Samples whose gyro reading can be trusted.

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


def turn_ratio(t_src, w_src, dt_src, tg, wg, min_rate=0.1):
    """Rotation measured by the gyro divided by the rotation measured by a source, over the
    source's turning windows (|yaw rate| above min_rate) that the gyro fully covers.
    A gyro scale-factor error shows as a ratio away from 1 that is the same in every turn.
    Returns (ratio, seconds of turning) or (None, 0)."""
    if len(tg) < 100 or len(t_src) == 0:
        return None, 0.0
    hg = np.concatenate([[0.0], np.cumsum(0.5 * (wg[1:] + wg[:-1]) * np.diff(tg))])
    a, b = t_src - dt_src / 2, t_src + dt_src / 2
    sel = (a >= tg[0]) & (b <= tg[-1]) & (np.abs(w_src) > min_rate)
    gap = np.diff(tg).max(initial=0.0)
    if gap > 0.1:   # skip windows that straddle a hole in the gyro data
        ia, ib = np.searchsorted(tg, a), np.searchsorted(tg, b)
        expected = (b - a) / max(float(np.median(np.diff(tg))), 1e-4)
        sel &= (ib - ia) >= 0.5 * expected
    if sel.sum() < 5:
        return None, 0.0
    rot_g = np.interp(b[sel], tg, hg) - np.interp(a[sel], tg, hg)
    rot_s = w_src[sel] * dt_src[sel]
    return float(np.sum(rot_g * np.sign(rot_s)) / np.sum(np.abs(rot_s))), float(np.sum(dt_src[sel]))


def regularize_times(t):
    """Snap a periodic sensor's timestamps to a uniform grid, allowing for dropped frames.

    Bag record times jitter: a message reaches the recorder late, then the next one
    arrives almost at once. Speeds computed over such intervals are wrong in both
    directions and the errors do not cancel when integrated. The sensor itself fires at
    a fixed period, so each sample is assigned its frame number and the times are
    refitted as start + frame * period. Returns (times, info), or (t, None) when the
    series is not periodic enough to do this safely.
    """
    t = np.asarray(t, float)
    if len(t) < 20:
        return t, None
    d = np.diff(t)
    T = float(np.median(d))
    if T <= 0:
        return t, None
    inc = np.maximum(np.round(d / T).astype(int), 0)
    for j in np.flatnonzero(inc == 0):
        # a late message followed by an early one is one delay, not a dropped frame
        for back in range(j - 1, max(-1, j - 6), -1):
            if inc[back] >= 2:
                inc[back] -= 1
                break
        inc[j] = 1
    k = np.concatenate([[0], np.cumsum(inc)]).astype(float)
    A = np.column_stack([np.ones_like(k), k])
    coef, *_ = np.linalg.lstsq(A, t, rcond=None)
    res = t - A @ coef
    mad = float(np.median(np.abs(res - np.median(res))))
    keep = np.abs(res - np.median(res)) <= max(4.0 * 1.4826 * mad, 1e-3)
    coef, *_ = np.linalg.lstsq(A[keep], t[keep], rcond=None)
    res = t - A @ coef
    # late messages are genuine outliers (up to about a period); a wrong frame count would
    # instead shift every later sample by a whole period
    if coef[1] <= 0 or np.mean(np.abs(res) < 0.3 * coef[1]) < 0.9:
        return t, None
    return A @ coef, {"period_s": float(coef[1]), "residual_rms_s": float(np.sqrt(np.mean(res ** 2))),
                      "max_residual_s": float(np.max(np.abs(res))), "gaps": int(k[-1] - (len(t) - 1))}


def mount_correction(inc):
    """Yaw between the sensor's frame and the direction of travel, from straight driving."""
    if len(inc) == 0:
        return 0.0, 0
    speed = np.hypot(inc[:, 1], inc[:, 2])
    straight = (speed > 0.15) & (np.abs(inc[:, 3]) < 0.05)
    if straight.sum() < 10:
        return 0.0, int(straight.sum())
    ang = np.arctan2(inc[straight, 2], inc[straight, 1])
    ang = np.where(inc[straight, 1] < 0, ang - math.pi, ang)
    ang = (ang + math.pi) % (2 * math.pi) - math.pi
    return float(np.median(ang)), int(straight.sum())


def estimate_lag(t_src, w_src, t_ref, w_ref, max_lag=0.5, fs=100.0):
    """Lag L (s) such that source(t) matches reference(t - L), by cross-correlating yaw rates.

    Returns (lag, correlation, sign). sign is -1 when the source turns the opposite way
    to the reference (a sign-convention difference), in which case the lag and the
    correlation are those of the negated source.
    """
    t0 = max(t_src[0], t_ref[0]) + max_lag
    t1 = min(t_src[-1], t_ref[-1]) - max_lag
    if t1 - t0 < 10.0:
        return 0.0, None, 1
    grid = np.arange(t0, t1, 1.0 / fs)
    a = np.interp(grid, t_src, w_src)
    if np.std(a) < 0.02:
        return 0.0, None, 1
    lags = np.arange(-max_lag, max_lag + 1e-9, 1.0 / fs)
    corr = np.array([np.corrcoef(a, np.interp(grid - L, t_ref, w_ref))[0, 1] for L in lags])
    sign = -1 if -np.nanmin(corr) > np.nanmax(corr) else 1
    corr = sign * corr
    i = int(np.nanargmax(corr))
    best = float(lags[i])
    if 0 < i < len(lags) - 1:
        c0, c1, c2 = corr[i - 1], corr[i], corr[i + 1]
        den = c0 - 2 * c1 + c2
        if den < 0:
            best += 0.5 * (c0 - c2) / den / fs
    return best, float(corr[i]), sign


class PlanarEKF:
    """Planar EKF. State: x, y, heading, speed v, yaw rate w, gyro bias b, gyro scale error s,
    then one heading copy ("clone") per relative-heading source: the heading at the start of
    that source's current interval, so its measured heading change can be compared with
    the filter's own heading change over the same interval."""
    X, Y, TH, V, W, B, S = range(7)

    def __init__(self, n_clones, sigma_bias0, sigma_scale0):
        self.n = 7 + n_clones
        self.x = np.zeros(self.n)
        self.P = np.zeros((self.n, self.n))
        self.P[:7, :7] = np.diag([1e-6, 1e-6, 1e-6, 0.1 ** 2, 0.1 ** 2, sigma_bias0 ** 2, sigma_scale0 ** 2])
        self.clone_of = [None] * n_clones    # interval each clone was taken for
        self.eye = np.eye(self.n)
        self.q = np.zeros(self.n)
        self.q[:7] = [1e-8, 1e-8, 1e-8, SIGMA_ACC ** 2, SIGMA_ALPHA ** 2, SIGMA_BIAS_RW ** 2, SIGMA_SCALE_RW ** 2]

    def predict(self, dt):
        if dt <= 0:
            return
        n = max(1, int(math.ceil(dt / 0.02)))
        h = dt / n
        Q = np.diag(self.q * h)
        for _ in range(n):
            x = self.x
            v, w = x[3], x[4]
            thm = x[2] + 0.5 * w * h
            c, s = math.cos(thm), math.sin(thm)
            x[0] += v * c * h
            x[1] += v * s * h
            x[2] += w * h
            F = self.eye.copy()
            F[0, 2], F[0, 3], F[0, 4] = -v * s * h, c * h, -0.5 * v * s * h * h
            F[1, 2], F[1, 3], F[1, 4] = v * c * h, s * h, 0.5 * v * c * h * h
            F[2, 4] = h
            self.P = F @ self.P @ F.T + Q

    def update(self, z, h, H, R):
        """Measurement z with prediction h(x) and Jacobian H; chi-square gated at 99 %."""
        y = np.atleast_1d(z - h)
        S = H @ self.P @ H.T + R
        Si = np.linalg.inv(S)
        nis = float(y @ Si @ y)
        if nis > CHI2_99[len(y)]:
            return nis, False
        K = self.P @ H.T @ Si
        self.x = self.x + K @ y
        I_KH = self.eye - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return nis, True

    def clone(self, j, interval):
        c = 7 + j
        self.x[c] = self.x[2]
        self.P[c, :] = self.P[2, :]
        self.P[:, c] = self.P[:, 2]
        self.P[c, c] = self.P[2, 2]
        self.clone_of[j] = interval

    def update_gyro(self, z, sigma):
        x = self.x
        H = np.zeros((1, self.n))
        H[0, 4], H[0, 5], H[0, 6] = 1.0 + x[6], 1.0, x[4]
        return self.update(np.array([z]), np.array([(1.0 + x[6]) * x[4] + x[5]]), H, np.array([[sigma ** 2]]))

    def update_speed(self, z, sigma):
        H = np.zeros((1, self.n))
        H[0, 3] = 1.0
        return self.update(np.array([z]), np.array([self.x[3]]), H, np.array([[sigma ** 2]]))

    def update_speed_rate(self, z, sig):
        H = np.zeros((2, self.n))
        H[0, 3] = H[1, 4] = 1.0
        return self.update(np.asarray(z), self.x[[3, 4]], H, np.diag(np.asarray(sig) ** 2))

    def update_heading_change(self, j, z, sigma):
        c = 7 + j
        H = np.zeros((1, self.n))
        H[0, 2], H[0, c] = 1.0, -1.0
        return self.update(np.array([z]), np.array([self.x[2] - self.x[c]]), H, np.array([[sigma ** 2]]))


def main():
    ap = argparse.ArgumentParser(description="Planar EKF over each sensor combination of a Scout 2.0 run.")
    ap.add_argument("path", type=Path, help="bag folder, or its results_<bag name> folder")
    ap.add_argument("--results", type=Path, default=None, help="results folder (default: results_<bag name>)")
    ap.add_argument("--configs", nargs="*", default=list(CONFIGS), help=f"subset of: {' '.join(CONFIGS)}")
    ap.add_argument("--no-lag", action="store_true", help="do not estimate time offsets")
    ap.add_argument("--heading-window", type=float, default=HEADING_WINDOW_S,
                    help="interval for LiDAR and visual heading-change measurements, seconds")
    ap.add_argument("--estimate-scale", action="store_true",
                    help="estimate the gyro scale error online against the LiDAR / camera heading (off by default: "
                         "it needs an unbiased rotation reference, which KISS-ICP without deskewing is not)")
    ap.add_argument("--gyro-scale", type=float, default=1.0,
                    help="multiply the gyro yaw rate by this (a scale calibration measured on a different run)")
    args = ap.parse_args()

    res, bag_name = resolve_results(args.path)
    if args.results:
        res = args.results.expanduser().resolve()
    if not res.exists():
        sys.exit(f"Results folder {res} not found. Run kiss_vs_wheel.py and extract_signals.py first.")
    calib = json.loads((res / "calibration.json").read_text(encoding="utf-8")) if (res / "calibration.json").exists() else {}
    still = calib.get("stationary_window_s")
    print(f"Results folder: {res}")

    # ---------------- measurement preparation
    src = {}        # name -> dict(t, z (n x k), sig (n x k), info)
    info = {}

    def still_std(t, vals):
        if not still:
            return None
        m = (t >= still[0]) & (t <= still[1])
        return float(np.std(vals[m])) if m.sum() >= 5 else None

    # wheel
    if (res / "wheel.csv").exists():
        d = load_csv(res / "wheel.csv")
        inc = average_rates(d["time_s"], d["v_mps"], d["w_radps"])
        origin = "wheel.csv (driver speed and yaw rate)"
    elif (res / "wheel_odom.csv").exists():
        d = load_csv(res / "wheel_odom.csv")
        tw, reg = regularize_times(d["time_s"])
        inc = velocity_from_pose(tw, d["x_m"], d["y_m"], d["yaw_rad"])
        origin = ("wheel_odom.csv (pose differences over record times"
                  + (", snapped to the message period" if reg else "") + "; run extract_signals.py for driver speeds)")
    else:
        inc = np.zeros((0, 5))
    if len(inc):
        v, w = inc[:, 1], inc[:, 3]
        sv = 0.03 + 0.05 * np.abs(v) + 0.10 * np.abs(w)
        sw = 0.02 + 0.30 * np.abs(w)   # skid-steer yaw rate: slip error grows with turn rate
        src["wheel"] = dict(t=inc[:, 0], z=np.column_stack([v, w]), sig=np.column_stack([sv, sw]), dt=inc[:, 4])
        info["wheel"] = {"source": origin, "measurements": len(inc),
                         "sigma_model": "v: 0.03 + 0.05|v| + 0.10|w| m/s; w: 0.02 + 0.30|w| rad/s"}

    # gyro
    if (res / "imu.csv").exists():
        d = load_csv(res / "imu.csv")
        b0 = float(calib.get("gyro_bias_up_radps") or 0.0)
        std_still = calib.get("yaw_rate_std_still_radps")
        sg = max(0.01, 3.0 * std_still) if std_still else 0.02   # driving vibration exceeds the still noise
        g3 = np.column_stack([d["gx_radps"], d["gy_radps"], d["gz_radps"]])
        ok = gyro_usable(d["record_s"], d["time_s"], g3)
        wg = args.gyro_scale * (d["yaw_rate_radps"] - b0)
        info["gyro"] = {"source": "imu.csv, yaw rate about the vertical", "measurements": int(ok.sum()),
                        "scale_applied": args.gyro_scale,
                        "initial_bias_radps": b0, "sigma_radps": sg,
                        "rate_hz": round((int(ok.sum()) - 1) / max(float(np.ptp(d["time_s"][ok])), 1e-3), 1) if ok.sum() > 1 else 0.0,
                        "usable_pct": round(100.0 * ok.mean(), 1)}
        if not ok.all():
            first_bad = float(d["record_s"][np.argmin(ok)] - d["record_s"][0])
            info["gyro"]["first_unusable_s"] = round(first_bad, 1)
            info["gyro"]["unusable_samples"] = int((~ok).sum())
            if ok.mean() < 0.99:
                print(f"  WARNING: {int((~ok).sum())} gyro samples unusable (stale stamps, zeros or frozen values), "
                      f"first at {first_bad:.1f} s; {100 * ok.mean():.0f}% usable. Configurations with the gyro use "
                      f"their other sensors where it is missing.")
        if ok.sum() >= 100:
            src["gyro"] = dict(t=d["time_s"][ok], z=wg[ok][:, None], sig=np.full((int(ok.sum()), 1), sg))

    # LiDAR and visual odometry, from pose tracks
    for name, fname in (("lidar", "kiss_icp_poses.csv"), ("visual", "vo.csv")):
        if not (res / fname).exists():
            continue
        d = load_csv(res / fname)
        yaw = np.unwrap(d["yaw_rad"])
        valid = np.ones(len(yaw), bool)
        if "status" in d:
            valid = d["status"] != "lost"
        t = d["time_s"]
        if name == "lidar":
            # kiss_icp_poses.csv carries bag record times. Use the scans' header stamps where
            # extract_signals.py found them, then snap to the scan period (10 Hz).
            tnote = "bag record times"
            if (res / "lidar_stamps.csv").exists():
                ls = load_csv(res / "lidar_stamps.csv")
                j = np.clip(np.searchsorted(ls["record_s"], t), 1, len(ls["record_s"]) - 1)
                j = np.where(np.abs(ls["record_s"][j - 1] - t) < np.abs(ls["record_s"][j] - t), j - 1, j)
                hit = np.abs(ls["record_s"][j] - t) < 5e-6
                if hit.mean() > 0.95:
                    t = np.where(hit, ls["time_s"][j], t)
                    tnote = f"scan header stamps from lidar_stamps.csv ({100 * hit.mean():.1f}% matched)"
            t_reg, reg = regularize_times(t)
        else:
            vos = json.loads((res / "vo_summary.json").read_text(encoding="utf-8")) if (res / "vo_summary.json").exists() else {}
            tnote = f"image {vos.get('time_base', 'header')} times"
            t_reg, reg = regularize_times(t) if vos.get("time_base") == "record" else (t, None)
        if reg:
            t = t_reg
            tnote += (f", snapped to a {1 / reg['period_s']:.3f} Hz grid (residual {1000 * reg['residual_rms_s']:.1f} ms RMS, "
                      f"max {1000 * reg['max_residual_s']:.0f} ms, {reg['gaps']} missing frames)")
        inc = velocity_from_pose(t, d["x_m"], d["y_m"], yaw, valid)
        if len(inc) < 10:
            continue
        delta, n_straight = mount_correction(inc)
        c, s = math.cos(delta), math.sin(delta)
        vf = c * inc[:, 1] + s * inc[:, 2]
        vlat = -s * inc[:, 1] + c * inc[:, 2]
        w = inc[:, 3]
        floor_v, floor_w = (0.02, 0.01) if name == "lidar" else (0.03, 0.015)
        sv_still, sw_still = still_std(inc[:, 0], vf), still_std(inc[:, 0], w)
        sv0 = max(floor_v, 1.5 * sv_still) if sv_still is not None else 2 * floor_v
        sw0 = max(floor_w, 1.5 * sw_still) if sw_still is not None else 2 * floor_w
        sv = sv0 + 0.05 * np.abs(vf)
        sw = sw0 + 0.05 * np.abs(w)
        frac = HEADING_FRAC[name]
        hinc = velocity_from_pose(t, d["x_m"], d["y_m"], yaw, valid, window=args.heading_window)
        dth = hinc[:, 3] * hinc[:, 4]
        # pose noise does not accumulate within an interval: the floor is the 0.2 s value
        sdth = np.sqrt((sw0 * WINDOW_S) ** 2 + (frac * np.abs(dth)) ** 2)
        src[name] = dict(t=inc[:, 0], z=np.column_stack([vf, w]), sig=np.column_stack([sv, sw]), dt=inc[:, 4],
                         ta=hinc[:, 5], tb=hinc[:, 6], dth=dth, sig_dth=sdth)
        straight = (np.abs(vf) > 0.15) & (np.abs(w) < 0.05)
        info[name] = {"source": fname, "timestamps": tnote, "measurements": len(inc),
                      "skipped_intervals_lost_tracking": int((~valid).sum()),
                      "mount_correction_deg": round(math.degrees(delta), 2),
                      "straight_samples": n_straight,
                      "sideways_speed_rms_straight_mps": round(float(np.sqrt(np.mean(vlat[straight] ** 2))), 4)
                      if straight.any() else None,
                      "sigma_model": f"v: {sv0:.3f} + 0.05|v| m/s; heading change over each interval: "
                                     f"{sw0 * WINDOW_S:.4f} rad combined with {100 * frac:.0f}% of the turn, "
                                     f"over {args.heading_window:.1f} s intervals"}

    if not src:
        sys.exit("No inputs found. Expected wheel.csv / imu.csv / kiss_icp_poses.csv / vo.csv in the results folder.")

    # ---------------- yaw-rate sign check and time offsets, relative to the gyro
    if "gyro" in src:
        tg, wg = src["gyro"]["t"], src["gyro"]["z"][:, 0]
        found = {}
        for name in ("wheel", "lidar", "visual"):
            if name in src:
                found[name] = estimate_lag(src[name]["t"], src[name]["z"][:, 1], tg, wg)
        opposite = [n for n, (_, c, s) in found.items() if c is not None and c >= 0.5 and s < 0]
        agree = [n for n, (_, c, s) in found.items() if c is not None and c >= 0.5 and s > 0]
        if len(opposite) >= 2 and len(opposite) > len(agree):
            src["gyro"]["z"] = -src["gyro"]["z"]
            info["gyro"]["yaw_rate_sign_flipped"] = True
            print("  WARNING: the gyro turns opposite to the other sensors; its sign was flipped for fusion.")
            found = {n: (lag, c, -s) for n, (lag, c, s) in found.items()}
        for name, (lag, corr, sign) in found.items():
            if corr is not None and corr >= 0.5 and sign < 0:
                src[name]["z"][:, 1] = -src[name]["z"][:, 1]
                if "dth" in src[name]:
                    src[name]["dth"] = -src[name]["dth"]
                info[name]["yaw_rate_sign_flipped"] = True
                print(f"  WARNING: {name} yaw rate turns opposite to the gyro; its sign was flipped for fusion. "
                      f"Check that sensor's frame or driver convention.")
            applied = not args.no_lag and corr is not None and corr >= 0.5 and abs(lag) < 0.48
            if applied:
                for key in ("t", "ta", "tb"):
                    if key in src[name]:
                        src[name][key] = src[name][key] - lag
            info[name]["time_offset_s"] = round(lag, 3) if applied else 0.0
            info[name]["time_offset_correlation"] = round(corr, 3) if corr is not None else None
        # gyro scale check: rotation in turns, gyro against each other source
        for name in ("lidar", "visual", "wheel"):
            if name in src:
                r, secs = turn_ratio(src[name]["t"], src[name]["z"][:, 1], src[name]["dt"],
                                     src["gyro"]["t"], src["gyro"]["z"][:, 0])
                if r is not None:
                    info[name]["gyro_turn_ratio"] = round(r, 4)
                    info[name]["turning_s"] = round(secs, 1)
                    print(f"  gyro rotation in turns = {100 * r:.1f}% of {name} rotation ({secs:.0f} s of turning)")

    for name in src:
        print(f"  {name:7s} {len(src[name]['t']):6d} measurements")

    # ---------------- run each configuration
    sigma_b0 = 0.002 if still else 0.01
    summary = {"bag": bag_name, "sources": info, "configs": {},
               "settings": {"window_s": WINDOW_S, "gate": "chi-square 99 %", "sigma_acc_mps2": SIGMA_ACC,
                            "sigma_alpha_radps2": SIGMA_ALPHA, "sigma_bias_rw": SIGMA_BIAS_RW,
                            "initial_bias_sigma_radps": sigma_b0, "gyro_scale_estimated": bool(args.estimate_scale),
                            "gyro_scale_applied": args.gyro_scale, "heading_window_s": args.heading_window,
                            "heading_change_fraction": HEADING_FRAC}}
    lines = [f"Bag: {bag_name}", ""]
    for cfg in args.configs:
        if cfg not in CONFIGS:
            print(f"  unknown configuration {cfg}, skipped")
            continue
        need = CONFIGS[cfg]
        missing = [n for n in need if n not in src]
        if missing:
            print(f"  {cfg}: skipped, no {', '.join(missing)} data")
            continue
        # events: gyro and wheel as rates; LiDAR and visual as speed (interval middle) plus
        # heading change (interval end, against the heading cloned at the interval start)
        rel = [n for n in need if n in ("lidar", "visual")]
        rel_idx = {n: j for j, n in enumerate(rel)}
        ev_t, ev_pri, ev_kind, ev_src, ev_row = [], [], [], [], []

        def add(times, pri, kind, k):
            ev_t.append(np.asarray(times, float))
            ev_pri.append(np.full(len(times), pri))
            ev_kind.append(np.full(len(times), kind))
            ev_src.append(np.full(len(times), k))
            ev_row.append(np.arange(len(times)))

        for k, name in enumerate(need):
            sname = src[name]
            if name == "gyro":
                add(sname["t"], 0, 0, k)
            elif name == "wheel":
                add(sname["t"], 0, 1, k)
            else:
                add(sname["t"], 0, 2, k)
                add(sname["tb"], 0, 3, k)
                add(sname["ta"], 1, 4, k)     # at a shared instant the old interval closes first
        ev_t, ev_pri, ev_kind = np.concatenate(ev_t), np.concatenate(ev_pri), np.concatenate(ev_kind)
        ev_src, ev_row = np.concatenate(ev_src), np.concatenate(ev_row)
        order = np.lexsort((ev_pri, ev_t))
        # the gyro scale error is observable only against a rotation reference (LiDAR or camera);
        # the wheel's yaw rate is too slip-affected to calibrate it
        scale0 = SIGMA_SCALE0 if ("gyro" in need and rel and args.estimate_scale) else 0.0
        ekf = PlanarEKF(len(rel), sigma_b0, scale0)
        t_prev = ev_t[order[0]]
        traj, gate = [], []
        last_out = -1e18
        n_meas = {name: 0 for name in need}
        n_rej = {name: 0 for name in need}
        kind_name = {0: "yaw rate", 1: "speed and yaw rate", 2: "speed", 3: "heading change"}

        def snapshot(t):
            x, P = ekf.x, ekf.P
            return (t, x[0], x[1], x[2], x[3], x[4], x[5], x[6], math.sqrt(P[2, 2]), math.sqrt(P[0, 0] + P[1, 1]))

        for e in order:
            t, kind, name, r = ev_t[e], ev_kind[e], need[ev_src[e]], ev_row[e]
            ekf.predict(t - t_prev)
            t_prev = t
            sname = src[name]
            if kind == 4:
                ekf.clone(rel_idx[name], r)
                continue
            if kind == 0:
                nis, ok = ekf.update_gyro(sname["z"][r, 0], sname["sig"][r, 0])
                dof = 1
            elif kind == 1:
                nis, ok = ekf.update_speed_rate(sname["z"][r], sname["sig"][r])
                dof = 2
            elif kind == 2:
                nis, ok = ekf.update_speed(sname["z"][r, 0], sname["sig"][r, 0])
                dof = 1
            else:
                j = rel_idx[name]
                if ekf.clone_of[j] != r:
                    continue
                nis, ok = ekf.update_heading_change(j, sname["dth"][r], sname["sig_dth"][r])
                dof = 1
            n_meas[name] += 1
            if not ok:
                n_rej[name] += 1
            if kind != 0 or not ok:
                gate.append((t, name, kind_name[kind], nis, dof, int(ok)))
            if t - last_out >= 0.02:
                traj.append(snapshot(t))
                last_out = t
        traj.append(snapshot(t_prev))

        with open(res / f"traj_{cfg}.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "x_m", "y_m", "yaw_rad", "v_mps", "w_radps", "gyro_bias_radps", "gyro_scale_error",
                         "sigma_yaw_rad", "sigma_xy_m"])
            for row in traj:
                wr.writerow([f"{row[0]:.6f}", f"{row[1]:.4f}", f"{row[2]:.4f}", f"{row[3]:.5f}", f"{row[4]:.4f}",
                             f"{row[5]:.5f}", f"{row[6]:.6f}", f"{row[7]:.5f}", f"{row[8]:.5f}", f"{row[9]:.4f}"])
        with open(res / f"gating_{cfg}.csv", "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["time_s", "sensor", "measurement", "nis", "dof", "accepted"])
            for row in gate:
                wr.writerow([f"{row[0]:.6f}", row[1], row[2], f"{row[3]:.3f}", row[4], row[5]])

        end = np.array(traj[-1][1:3])
        per = {name: {"measurements": n_meas[name], "rejected": n_rej[name],
                      "rejected_pct": round(100.0 * n_rej[name] / max(1, n_meas[name]), 2)} for name in need}
        summary["configs"][cfg] = {"sensors": list(need), "duration_s": round(float(traj[-1][0] - traj[0][0]), 2),
                                   "final_xy_m": [round(float(end[0]), 3), round(float(end[1]), 3)],
                                   "final_distance_from_start_m": round(float(np.linalg.norm(end)), 3),
                                   "final_heading_deg": round(math.degrees(traj[-1][3]), 2),
                                   "final_gyro_bias_radps": round(float(traj[-1][6]), 6) if "gyro" in need else None,
                                   "final_gyro_scale_error": round(float(traj[-1][7]), 5) if "gyro" in need else None,
                                   "gating": per}
        rej = ", ".join(f"{n} {per[n]['rejected_pct']:.1f}%" for n in need)
        line = (f"{cfg:22s} end {np.linalg.norm(end):6.2f} m from start, heading {math.degrees(traj[-1][3]):+8.1f} deg, "
                f"rejected: {rej}")
        print("  " + line)
        lines.append(line)

    lines += ["", "Sources:"]
    for name, d in info.items():
        lines.append(f"  {name}: " + ", ".join(f"{k} {v}" for k, v in d.items()))
    (res / "ekf_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (res / "ekf_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nResults written to {res}")


if __name__ == "__main__":
    main()
