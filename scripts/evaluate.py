#!/usr/bin/env python3
"""Drift metrics and figures for every odometry estimate of one Scout 2.0 run.

Reads from results_<bag name>:
    wheel_odom.csv, kiss_icp_poses.csv     kiss_vs_wheel.py
    zed_odom.csv                           extract_signals.py
    vo.csv, vo_summary.json                visual_odometry.py
    traj_<config>.csv, gating_all.csv,     fuse_ekf.py
    ekf_summary.json
and writes:
    metrics.csv, metrics.md                one row per estimate
    benchmark_tracks.png                   top-down tracks, one panel per sensor family
    benchmark_heading.png                  heading over time, same panels
    benchmark_drift.png                    error per estimate
    benchmark_gating.png                   consistency check of the all-sensor filter

There is no ground truth. The metrics use what the run itself guarantees:
    closure error      a run that ends on its start mark should end at (0, 0);
                       reported in metres and as a percentage of the reference path
    heading reversal   the outbound and return legs of one straight corridor are 180 deg
                       apart; each leg's direction is its principal axis (PCA)
    known distance     a run driven over a taped distance; straight-line displacement
                       from start to end against the tape

Presets exist for hallway_04_cmd (closed loop, legs at 45-74 s and 146-192 s) and
straight_5m (5.00 m). Other runs: pass --loop, --outbound/--return or --known-distance.

Usage:
    python evaluate.py PATH_TO_BAG_FOLDER        (or the results_<bag name> folder)
    python evaluate.py bags\\hallway_03 --loop
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

PRESETS = {
    "hallway_04_cmd": {"loop": True, "outbound": (45.0, 74.0), "return": (146.0, 192.0)},
    "straight_5m": {"known_distance": 5.00},
}

# key, label, file, kind, sensors
METHODS = [
    ("wheel_native", "Wheel (driver odometry)", "wheel_odom.csv", "native", "wheel"),
    ("lidar_native", "LiDAR (KISS-ICP)", "kiss_icp_poses.csv", "native", "LiDAR"),
    ("visual_native", "Visual (RGB-D VO)", "vo.csv", "native", "camera"),
    ("zed_vio", "Visual-inertial (ZED SDK)", "zed_odom.csv", "native", "camera, IMU"),
    ("wheel", "Wheel", "traj_wheel.csv", "ekf", "wheel"),
    ("wheel_inertial", "Wheel-inertial", "traj_wheel_inertial.csv", "ekf", "wheel, gyro"),
    ("visual", "Visual", "traj_visual.csv", "ekf", "camera"),
    ("visual_inertial", "Visual-inertial", "traj_visual_inertial.csv", "ekf", "camera, gyro"),
    ("lidar", "LiDAR", "traj_lidar.csv", "ekf", "LiDAR"),
    ("lidar_inertial", "LiDAR-inertial", "traj_lidar_inertial.csv", "ekf", "LiDAR, gyro"),
    ("visual_lidar_inertial", "Visual-LiDAR-inertial", "traj_visual_lidar_inertial.csv", "ekf", "camera, LiDAR, gyro"),
    ("all", "All sensors", "traj_all.csv", "ekf", "wheel, camera, LiDAR, gyro"),
]

# reference palette (dataviz skill), light surface
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
CRITICAL = "#d03b3b"
REF_GRAY = "#c3c2b7"

PANELS = [
    ("Wheel", [("wheel", BLUE), ("wheel_inertial", ORANGE)]),
    ("Visual", [("visual", BLUE), ("visual_inertial", ORANGE), ("zed_vio", AQUA)]),
    ("LiDAR", [("lidar", BLUE), ("lidar_inertial", ORANGE)]),
    ("Combined", [("visual_lidar_inertial", ORANGE), ("all", AQUA)]),
]


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


def path_length_1s(t, xy):
    """Path length from positions sampled once per second (suppresses scan-to-scan jitter)."""
    grid = np.arange(t[0], t[-1], 1.0)
    if len(grid) < 2:
        return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    g = np.column_stack([np.interp(grid, t, xy[:, 0]), np.interp(grid, t, xy[:, 1])])
    g = np.vstack([g, xy[-1]])
    return float(np.sum(np.linalg.norm(np.diff(g, axis=0), axis=1)))


def leg_direction(xy):
    c = xy - xy.mean(axis=0)
    _, _, vt = np.linalg.svd(c, full_matrices=False)
    d = vt[0]
    if np.dot(xy[-1] - xy[0], d) < 0:
        d = -d
    return d


def best_rotation_2d(a, b):
    num = np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])
    den = np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])
    return math.atan2(num, den)


def rotate(xy, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.column_stack([c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]])


def fmt(v, nd=2):
    return "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.{nd}f}"


def pick_font(font_manager):
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Segoe UI", "Helvetica Neue", "Arial", "Liberation Sans", "DejaVu Sans"):
        if name in have:
            return name
    return "sans-serif"


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelcolor=INK2, labelsize=8, length=3, width=0.8)
    ax.grid(True, color=GRID, linewidth=0.6, linestyle="-")
    ax.set_axisbelow(True)


def main():
    ap = argparse.ArgumentParser(description="Drift metrics and figures for the Scout 2.0 odometry benchmark.")
    ap.add_argument("path", type=Path, help="bag folder, or its results_<bag name> folder")
    ap.add_argument("--results", type=Path, default=None)
    ap.add_argument("--loop", action="store_true", help="the run ended on its start mark")
    ap.add_argument("--outbound", type=float, nargs=2, metavar=("START", "END"),
                    help="outbound leg window, seconds since the first LiDAR scan")
    ap.add_argument("--return", dest="ret", type=float, nargs=2, metavar=("START", "END"),
                    help="return leg window, seconds since the first LiDAR scan")
    ap.add_argument("--known-distance", type=float, default=None, help="taped start-to-end distance, metres")
    ap.add_argument("--ref-length", type=float, default=None,
                    help="path length to divide closure errors by (default: KISS-ICP path at 1 s sampling)")
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
    except ModuleNotFoundError as e:
        sys.exit(f"Missing package: {e.name}. Run: pip install matplotlib numpy")

    res, bag_name = resolve_results(args.path)
    if args.results:
        res = args.results.expanduser().resolve()
    if not res.exists():
        sys.exit(f"Results folder {res} not found.")
    preset = dict(PRESETS.get(bag_name, {}))
    if args.loop:
        preset["loop"] = True
    if args.outbound and args.ret:
        preset["outbound"], preset["return"] = tuple(args.outbound), tuple(args.ret)
    if args.known_distance is not None:
        preset["known_distance"] = args.known_distance
    loop = bool(preset.get("loop"))
    known = preset.get("known_distance")
    legs = (preset["outbound"], preset["return"]) if "outbound" in preset and "return" in preset else None

    # ---------------- load
    traj = {}
    for key, label, fname, kind, sensors in METHODS:
        f = res / fname
        if key == "wheel_native" and not f.exists():
            f = res / "wheel.csv"   # extract_signals.py output, when kiss_vs_wheel.py was not run
        if not f.exists():
            continue
        d = load_csv(f)
        if len(d.get("time_s", [])) < 10:
            continue
        t = d["time_s"]
        order = np.argsort(t, kind="stable")
        traj[key] = {"t": t[order], "xy": np.column_stack([d["x_m"], d["y_m"]])[order],
                     "yaw": np.unwrap(d["yaw_rad"][order]), "label": label, "kind": kind, "sensors": sensors}
    if not traj:
        sys.exit(f"No trajectories found in {res}.")

    ref_key = next((k for k in ("lidar_native", "lidar_inertial", "visual_lidar_inertial", "all", "wheel_inertial",
                                "visual_native", "wheel_native") if k in traj), next(iter(traj)))
    t_ref0, t_ref1 = traj[ref_key]["t"][0], traj[ref_key]["t"][-1]
    for tr in traj.values():
        m = (tr["t"] >= t_ref0 - 2.0) & (tr["t"] <= t_ref1 + 2.0)
        if m.sum() >= 10:
            for k in ("t", "xy", "yaw"):
                tr[k] = tr[k][m]
        tr["xy"] = tr["xy"] - tr["xy"][0]
        tr["yaw"] = tr["yaw"] - tr["yaw"][0]
        span = min(tr["t"][-1], t_ref1) - max(tr["t"][0], t_ref0)
        tr["coverage"] = max(0.0, span) / max(1e-6, t_ref1 - t_ref0)

    # ---------------- metrics
    ref_len = args.ref_length
    ref_len_note = "given on the command line"
    if ref_len is None:
        if "lidar_native" in traj:
            ref_len = path_length_1s(traj["lidar_native"]["t"], traj["lidar_native"]["xy"])
            ref_len_note = "KISS-ICP path length, positions sampled at 1 s"
        else:
            ref_len = float(np.median([path_length_1s(tr["t"], tr["xy"]) for tr in traj.values()]))
            ref_len_note = "median path length of all estimates, positions sampled at 1 s"

    rows = []
    for key, label, fname, kind, sensors in METHODS:
        if key not in traj:
            continue
        tr = traj[key]
        t, xy, yaw = tr["t"], tr["xy"], tr["yaw"]
        r = {"key": key, "estimate": label, "kind": kind, "sensors": sensors, "file": fname,
             "duration_s": float(t[-1] - t[0]), "coverage_pct": 100.0 * tr["coverage"],
             "path_length_m": path_length_1s(t, xy),
             "end_x_m": float(xy[-1, 0]), "end_y_m": float(xy[-1, 1]),
             "displacement_m": float(np.linalg.norm(xy[-1])),
             "heading_change_deg": float(math.degrees(yaw[-1]))}
        if loop:
            r["closure_error_m"] = r["displacement_m"]
            r["closure_error_pct"] = 100.0 * r["displacement_m"] / ref_len
        if known is not None:
            r["distance_error_m"] = r["displacement_m"] - known
            r["distance_error_pct"] = 100.0 * (r["displacement_m"] - known) / known
        if legs:
            ts = t - t_ref0
            (a0, a1), (b0, b1) = legs
            mo, mr = (ts >= a0) & (ts <= a1), (ts >= b0) & (ts <= b1)
            if mo.sum() >= 5 and mr.sum() >= 5:
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(leg_direction(xy[mo]) @ leg_direction(xy[mr]))))))
                r["heading_reversal_deg"] = ang
                r["reversal_error_deg"] = 180.0 - ang
        rows.append(r)

    # ---------------- context from the other scripts
    ekf = json.loads((res / "ekf_summary.json").read_text(encoding="utf-8")) if (res / "ekf_summary.json").exists() else {}
    vos = json.loads((res / "vo_summary.json").read_text(encoding="utf-8")) if (res / "vo_summary.json").exists() else {}
    for r in rows:
        g = ekf.get("configs", {}).get(r["key"], {}).get("gating")
        if g:
            r["rejected"] = "; ".join(f"{s} {v['rejected_pct']:.1f}%" for s, v in g.items())

    # ---------------- metrics.csv
    cols = ["estimate", "kind", "sensors", "duration_s", "coverage_pct", "path_length_m", "displacement_m",
            "closure_error_m", "closure_error_pct", "distance_error_m", "distance_error_pct",
            "heading_reversal_deg", "reversal_error_deg", "heading_change_deg", "end_x_m", "end_y_m", "rejected"]
    with open(res / "metrics.csv", "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(cols)
        for r in rows:
            wr.writerow([fmt(r[c], 4) if isinstance(r.get(c), float) else r.get(c, "") for c in cols])

    # ---------------- metrics.md
    dur = t_ref1 - t_ref0
    md = [f"# Odometry benchmark: {bag_name}", ""]
    md.append(f"Run length: {dur:.1f} s. Reference path length: {ref_len:.1f} m ({ref_len_note}).")
    kinds = []
    if loop:
        kinds.append("closed loop (ends on its start mark)")
    if legs:
        kinds.append(f"heading reversal legs at {legs[0][0]:.0f} to {legs[0][1]:.0f} s and "
                     f"{legs[1][0]:.0f} to {legs[1][1]:.0f} s after the first LiDAR scan")
    if known is not None:
        kinds.append(f"known start-to-end distance {known:.2f} m")
    md.append("Run type: " + ("; ".join(kinds) if kinds else "no reference given (comparison between estimates only)") + ".")
    md.append("")
    head = ["Estimate", "Sensors", "Path (m)"]
    if loop:
        head += ["Closure error (m)", "Closure error (% of path)"]
    if known is not None:
        head += ["Displacement (m)", "Error (m)", "Error (%)"]
    if legs:
        head += ["Heading reversal (deg)", "Reversal error (deg)"]
    head += ["Heading change (deg)", "Gate rejections"]
    for kind_name, title in (("native", "Native estimators"), ("ekf", "Common EKF, by sensor set")):
        sub = [r for r in rows if r["kind"] == kind_name]
        if not sub:
            continue
        md += [f"## {title}", "", "| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
        for r in sub:
            cells = [r["estimate"], r["sensors"], fmt(r["path_length_m"], 1)]
            if loop:
                cells += [fmt(r["closure_error_m"]), fmt(r["closure_error_pct"])]
            if known is not None:
                cells += [fmt(r["displacement_m"]), fmt(r["distance_error_m"], 2), fmt(r["distance_error_pct"], 1)]
            if legs:
                cells += [fmt(r.get("heading_reversal_deg"), 1), fmt(r.get("reversal_error_deg"), 1)]
            cells += [fmt(r["heading_change_deg"], 1), r.get("rejected", "")]
            md.append("| " + " | ".join(cells) + " |")
        md.append("")
    partial = [r["estimate"] for r in rows if r["coverage_pct"] < 95.0]
    if partial:
        md += [f"Partial coverage (less than 95 % of the run): {', '.join(partial)}. Their errors are not comparable.", ""]

    md += ["## Calibration and data quality", ""]
    for name, d in ekf.get("sources", {}).items():
        bits = []
        if "time_offset_s" in d:
            bits.append(f"time offset {d['time_offset_s'] * 1000:+.0f} ms relative to the gyro"
                        + (f" (correlation {d['time_offset_correlation']:.2f})" if d.get("time_offset_correlation") else ""))
        if "mount_correction_deg" in d:
            bits.append(f"mounting yaw correction {d['mount_correction_deg']:+.2f} deg")
        if "sideways_speed_rms_straight_mps" in d and d["sideways_speed_rms_straight_mps"] is not None:
            bits.append(f"sideways speed on straights {d['sideways_speed_rms_straight_mps']:.3f} m/s RMS")
        if "initial_bias_radps" in d:
            bits.append(f"initial gyro bias {math.degrees(d['initial_bias_radps']):+.3f} deg/s, "
                        f"noise {d['sigma_radps']:.4f} rad/s at {d.get('rate_hz', 0):.0f} Hz")
        if d.get("first_unusable_s") is not None:
            bits.append(f"**usable for {d.get('usable_pct', 0):.0f} % of samples only; unusable from "
                        f"{d['first_unusable_s']:.1f} s** (stale stamps, zeros or frozen values). After that point "
                        f"the configurations that include the gyro run on their other sensors")
        if "sigma_model" in d:
            bits.append(f"noise {d['sigma_model']}")
        md.append(f"- {name}: " + "; ".join(bits) + ".")
    if vos:
        sc = vos.get("status_counts", {})
        md.append(f"- visual odometry: {vos.get('frames')} frames, {sc.get('lost', 0)} lost "
                  f"({100 * vos.get('lost_fraction', 0):.1f} %), {sc.get('recovered', 0)} recovered, "
                  f"{vos.get('mean_inliers')} PnP inliers per frame, {vos.get('levelling')}.")
    md += ["", "## Definitions", "",
           "- Native estimators are the outputs of each method on its own: the Scout driver's wheel odometry, "
           "KISS-ICP, the RGB-D visual odometry in visual_odometry.py, and the ZED SDK's visual-inertial odometry "
           "(closed source, computed on the robot during the run).",
           "- The EKF rows come from one planar filter (fuse_ekf.py) run once per sensor set with identical settings. "
           "The wheel enters as speed and yaw rate; LiDAR and visual odometry as speed and heading change over "
           "0.2 s intervals; the gyro as yaw rate. The fusion is loosely coupled.",
           "- Path length uses positions sampled once per second, which suppresses scan-to-scan jitter.",
           "- Closure error is the distance between the start and end positions of a run that ended on its start mark. "
           "It measures accumulated drift, but errors that happen to cancel are not counted.",
           "- Heading reversal is the angle between the principal directions of the outbound and return legs of "
           "one straight corridor. The true value is 180 deg.",
           "- Heading change is the total rotation from start to end. It has no reference unless the robot's final "
           "orientation was recorded; differences between estimates show disagreement only.",
           "- Gate rejections are the share of each sensor's measurements that failed the 99 % chi-square "
           "innovation test in that filter.",
           "", "## Limitations", "",
           "- No ground truth. KISS-ICP serves as the plotting reference only.",
           "- One run per condition. Repeated runs are needed before differences of a few tenths of a metre are "
           "treated as real.",
           "- Raw sensor tracks (native rows) follow the sensor; EKF tracks follow the robot's turning centre. "
           "If the final heading differs from the initial heading, the sensor's lever arm adds to the native "
           "closure error.",
           "- The LiDAR has no per-point timestamps, so KISS-ICP runs without motion compensation.",
           ""]
    (res / "metrics.md").write_text("\n".join(md), encoding="utf-8")

    # ---------------- figures
    plt.rcParams.update({"font.family": pick_font(font_manager), "font.size": 9, "axes.titlesize": 10,
                         "axes.titleweight": "regular", "axes.titlecolor": INK, "axes.labelcolor": INK2,
                         "axes.labelsize": 9, "legend.fontsize": 8, "legend.frameon": False,
                         "figure.facecolor": SURFACE, "savefig.facecolor": SURFACE, "text.color": INK})

    # plotting alignment: rotate each track to the reference over the reference's first 15 m
    ref = traj[ref_key]
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref["xy"], axis=0), axis=1))])
    lim = min(15.0, 0.25 * cum[-1]) if cum[-1] > 0 else 0.0
    sel = cum <= max(lim, 1e-9)
    t_al = ref["t"][sel] if sel.sum() >= 20 else ref["t"][: max(20, len(ref["t"]) // 4)]
    ref_al = np.column_stack([np.interp(t_al, ref["t"], ref["xy"][:, 0]), np.interp(t_al, ref["t"], ref["xy"][:, 1])])
    for k, tr in traj.items():
        a = np.column_stack([np.interp(t_al, tr["t"], tr["xy"][:, 0]), np.interp(t_al, tr["t"], tr["xy"][:, 1])])
        tr["xy_plot"] = rotate(tr["xy"], best_rotation_2d(a, ref_al)) if k != ref_key and np.abs(a).sum() > 0 else tr["xy"]

    ref_label = f"{traj[ref_key]['label']} (reference track, not ground truth)"
    panels = [(title, [(k, c) for k, c in series if k in traj]) for title, series in PANELS]
    panels = [p for p in panels if p[1]]
    figs = []
    if panels:
        ncol = 2 if len(panels) > 1 else 1
        nrow = int(math.ceil(len(panels) / ncol))
        for kind in ("xy", "heading"):
            fig, axes = plt.subplots(nrow, ncol, figsize=(6.2 * ncol, 5.0 * nrow if kind == "xy" else 3.2 * nrow),
                                     squeeze=False)
            for ax, (title, series) in zip(axes.ravel(), panels):
                style_axes(ax)
                tl = ref["t"] - t_ref0
                if kind == "xy":
                    ax.plot(ref["xy_plot"][:, 0], ref["xy_plot"][:, 1], color=REF_GRAY, lw=3.0, solid_capstyle="round",
                            label=ref_label, zorder=1)
                else:
                    ax.plot(tl, np.degrees(ref["yaw"]), color=REF_GRAY, lw=3.0, label=ref_label, zorder=1)
                for k, color in series:
                    tr = traj[k]
                    lab = tr["label"] + (" (EKF)" if tr["kind"] == "ekf" else "")
                    if kind == "xy":
                        ax.plot(tr["xy_plot"][:, 0], tr["xy_plot"][:, 1], color=color, lw=2.0, solid_joinstyle="round",
                                solid_capstyle="round", label=lab, zorder=3)
                        ax.plot(*tr["xy_plot"][-1], "o", ms=7, color=color, mec=SURFACE, mew=2.0, zorder=4)
                    else:
                        ax.plot(tr["t"] - t_ref0, np.degrees(tr["yaw"]), color=color, lw=2.0, label=lab, zorder=3)
                if kind == "xy":
                    ax.plot(0, 0, "o", ms=8, color=INK, mec=SURFACE, mew=2.0, zorder=5, label="Start")
                    ax.set_aspect("equal", adjustable="datalim")
                    ax.set_xlabel("x (m)")
                    ax.set_ylabel("y (m)")
                else:
                    ax.set_xlabel("Time since first LiDAR scan (s)")
                    ax.set_ylabel("Heading change (deg)")
                ax.set_title(title, loc="left")
                ax.legend(loc="best")
            for ax in axes.ravel()[len(panels):]:
                ax.set_visible(False)
            sub = ("Top-down tracks. End points are dots; tracks rotated onto the reference for display."
                   if kind == "xy" else "Heading change since start.")
            fig.suptitle(f"{bag_name}: {sub}", x=0.01, ha="left", fontsize=10, color=INK)
            fig.tight_layout(rect=(0, 0, 1, 0.96))
            name = "benchmark_tracks.png" if kind == "xy" else "benchmark_heading.png"
            fig.savefig(res / name, dpi=150)
            plt.close(fig)
            figs.append(name)

    # drift summary: one bar per estimate
    metric_panels = []
    if loop:
        metric_panels.append(("closure_error_pct", "Closure error (% of path)", False))
    if known is not None:
        metric_panels.append(("distance_error_pct", f"Distance error vs {known:.2f} m (%)", True))
    if legs:
        metric_panels.append(("reversal_error_deg", "Heading reversal error (deg)", True))
    if metric_panels:
        shown = [r for r in rows if any(r.get(m[0]) is not None for m in metric_panels)]
        n = len(shown)
        fig, axes = plt.subplots(1, len(metric_panels), figsize=(4.6 * len(metric_panels) + 1.6, 0.36 * n + 1.3),
                                 squeeze=False, sharey=True)
        y = np.arange(n)[::-1]
        for ax, (col, title, signed) in zip(axes[0], metric_panels):
            style_axes(ax)
            ax.grid(True, axis="x", color=GRID, linewidth=0.6)
            ax.grid(False, axis="y")
            vals = np.array([r.get(col, np.nan) if r.get(col) is not None else np.nan for r in shown], dtype=float)
            colors = [MUTED if r["kind"] == "native" else BLUE for r in shown]
            ax.barh(y, np.nan_to_num(vals), height=0.42, color=colors, zorder=3)
            ax.axvline(0, color=AXIS, lw=0.8, zorder=2)
            span = np.nanmax(np.abs(vals)) if np.isfinite(vals).any() else 1.0
            for yy, v in zip(y, vals):
                if np.isfinite(v):
                    ax.text(v + (0.02 * span if v >= 0 else -0.02 * span), yy, f"{v:.1f}" if abs(v) >= 1 else f"{v:.2f}",
                            va="center", ha="left" if v >= 0 else "right", fontsize=8, color=INK2)
            lo = min(0.0, np.nanmin(vals)) if np.isfinite(vals).any() else 0.0
            hi = max(0.0, np.nanmax(vals)) if np.isfinite(vals).any() else 1.0
            pad = 0.18 * (hi - lo if hi > lo else 1.0)
            ax.set_xlim(lo - (pad if lo < 0 else 0), hi + pad)
            ax.set_title(title, loc="left")
        axes[0][0].set_yticks(y)
        axes[0][0].set_yticklabels([r["estimate"] + (" (EKF)" if r["kind"] == "ekf" else "") for r in shown])
        from matplotlib.patches import Patch
        fig.legend(handles=[Patch(color=MUTED, label="Native estimator"), Patch(color=BLUE, label="Common EKF")],
                   loc="lower right", ncol=2, frameon=False)
        fig.suptitle(f"{bag_name}: drift by estimate (no ground truth; see metrics.md)", x=0.01, ha="left", fontsize=10)
        fig.tight_layout(rect=(0, 0.05, 1, 0.95))
        fig.savefig(res / "benchmark_drift.png", dpi=150)
        plt.close(fig)
        figs.append("benchmark_drift.png")

    # gating of the all-sensor filter
    gate_cfg = next((c for c in ("all", "visual_lidar_inertial") if (res / f"gating_{c}.csv").exists()), None)
    if gate_cfg:
        g = load_csv(res / f"gating_{gate_cfg}.csv")
        sensors = [s for s in ("wheel", "visual", "lidar") if s in set(g["sensor"])]
        if sensors:
            tw = load_csv(res / f"traj_{gate_cfg}.csv") if (res / f"traj_{gate_cfg}.csv").exists() else None
            chi = {1: 6.635, 2: 9.21}
            fig, axes = plt.subplots(len(sensors), 1, figsize=(9.5, 2.1 * len(sensors) + 0.8), sharex=True, squeeze=False)
            for ax, s in zip(axes[:, 0], sensors):
                style_axes(ax)
                m = g["sensor"] == s
                tt, nis, acc = g["time_s"][m] - t_ref0, np.maximum(g["nis"][m], 1e-3), g["accepted"][m] > 0.5
                thr = chi.get(int(np.max(g["dof"][m])), 9.21) if "dof" in g else 9.21
                kinds = ", ".join(sorted(set(g["measurement"][m]))) if "measurement" in g else ""
                if tw is not None:
                    turning = np.abs(tw["w_radps"]) > 0.15
                    ttw = tw["time_s"] - t_ref0
                    ax.fill_between(ttw, 1e-3, 1e4, where=turning, color=GRID, alpha=0.6, lw=0, label="Turning (|yaw rate| > 0.15 rad/s)")
                ax.plot(tt[acc], nis[acc], "o", ms=3, color=BLUE, mew=0, label="Accepted")
                ax.plot(tt[~acc], nis[~acc], "o", ms=5, color=CRITICAL, mec=SURFACE, mew=1.0, label="Rejected")
                ax.axhline(thr, color=INK2, lw=0.8)
                ax.text(ax.get_xlim()[0] if False else tt.min(), thr * 1.25, "99% gate", fontsize=7, color=INK2)
                ax.set_yscale("log")
                ax.set_ylim(1e-3, max(1e2, float(np.nanmax(nis)) * 2))
                ax.set_ylabel("NIS")
                n_rej, n_all = int((~acc).sum()), int(len(acc))
                ax.set_title(f"{s}{' (' + kinds + ')' if kinds else ''}: {n_rej} of {n_all} measurements rejected "
                             f"({100 * n_rej / max(1, n_all):.1f}%)", loc="left")
            axes[0, 0].legend(loc="upper right", ncol=3)
            axes[-1, 0].set_xlabel("Time since first LiDAR scan (s)")
            fig.suptitle(f"{bag_name}: innovation consistency in the '{gate_cfg}' filter (normalised innovation squared; "
                         f"line = 99% gate)", x=0.01, ha="left", fontsize=10)
            fig.tight_layout(rect=(0, 0, 1, 0.95))
            fig.savefig(res / "benchmark_gating.png", dpi=150)
            plt.close(fig)
            figs.append("benchmark_gating.png")

    # ---------------- console
    print(f"Bag: {bag_name}   reference path {ref_len:.1f} m ({ref_len_note})")
    hdr = f"{'Estimate':34s} {'path m':>7s}"
    if loop:
        hdr += f" {'closure m':>10s} {'% path':>7s}"
    if known is not None:
        hdr += f" {'displ m':>8s} {'err %':>7s}"
    if legs:
        hdr += f" {'reversal':>9s}"
    print(hdr)
    for r in rows:
        line = f"{(r['estimate'] + (' (EKF)' if r['kind'] == 'ekf' else '')):34s} {r['path_length_m']:7.1f}"
        if loop:
            line += f" {r['closure_error_m']:10.2f} {r['closure_error_pct']:7.2f}"
        if known is not None:
            line += f" {r['displacement_m']:8.2f} {r['distance_error_pct']:7.1f}"
        if legs:
            line += f" {fmt(r.get('heading_reversal_deg'), 1):>9s}"
        print(line)
    print(f"\nWrote metrics.csv, metrics.md{', ' if figs else ''}{', '.join(figs)} to {res}")


if __name__ == "__main__":
    main()
