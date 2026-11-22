# Odometry benchmark

This document describes how odometry methods are compared on Scout 2.0 recordings: the
estimators, the shared filter, the metrics, the data problems found so far, and the
checks that the pipeline itself is correct. All scripts read ROS 2 bags directly on
Windows or Linux; no ROS installation is needed.

## Estimates compared

Native estimators (each method on its own):

| Estimate | Sensors | Produced by |
|---|---|---|
| Wheel (driver odometry) | wheel encoders | Scout driver, `/agilex_odom` |
| LiDAR (KISS-ICP) | RoboSense LiDAR | `scripts/kiss_vs_wheel.py` |
| Visual (RGB-D VO) | ZED X left image and registered depth | `scripts/visual_odometry.py` |
| Visual-inertial (ZED SDK) | ZED X cameras and IMU | ZED SDK positional tracking, `/zed/zed_node/odom`, computed on the robot |

Common EKF, one run per sensor set (`scripts/fuse_ekf.py`). All sets use the same
filter, motion model and noise settings, so differences come from the sensors alone:

| Configuration | Sensors |
|---|---|
| `wheel` | wheel speed and yaw rate |
| `wheel_inertial` | wheel + ZED IMU gyro |
| `visual` | visual odometry |
| `visual_inertial` | visual odometry + gyro |
| `lidar` | KISS-ICP |
| `lidar_inertial` | KISS-ICP + gyro |
| `visual_lidar_inertial` | visual odometry + KISS-ICP + gyro |
| `all` | wheel + visual odometry + KISS-ICP + gyro |

## Pipeline

Run from the folder that contains `bags\` and `scout2-autonomy\` (on the lab laptop,
`Documents`). Each script writes into `results_<bag name>`, the folder
`kiss_vs_wheel.py` already uses.

| Order | Script | Main outputs |
|---|---|---|
| 1 | `kiss_vs_wheel.py` | `kiss_icp_poses.csv`, `wheel_odom.csv` |
| 2 | `extract_signals.py` | `wheel.csv`, `imu.csv`, `zed_odom.csv`, `lidar_stamps.csv`, `camera_info.json`, `tf_static.json`, `calibration.json`, `extract_summary.txt` |
| 3 | `visual_odometry.py` | `vo.csv`, `vo_summary.txt`, `vo_summary.json` |
| 4 | `fuse_ekf.py` | `traj_<config>.csv`, `gating_<config>.csv`, `ekf_summary.txt`, `ekf_summary.json` |
| 5 | `evaluate.py` | `metrics.md`, `metrics.csv`, `benchmark_tracks.png`, `benchmark_heading.png`, `benchmark_drift.png`, `benchmark_gating.png` |

Commands, one bag:

```
py -3.12 scout2-autonomy\scripts\kiss_vs_wheel.py bags\hallway_04_cmd
```
```
py -3.12 scout2-autonomy\scripts\extract_signals.py bags\hallway_04_cmd
```
```
py -3.12 scout2-autonomy\scripts\visual_odometry.py bags\hallway_04_cmd
```
```
py -3.12 scout2-autonomy\scripts\fuse_ekf.py bags\hallway_04_cmd
```
```
py -3.12 scout2-autonomy\scripts\evaluate.py bags\hallway_04_cmd
```

Topics a bag does not contain are skipped, and configurations whose inputs are missing
are not run. Requirements: `pip install rosbags numpy opencv-python matplotlib kiss-icp`.

Read `extract_summary.txt` after every new recording. It reports the timing of every
sensor and flags unusable IMU data (see Data problems found).

## Method

### Timestamps

Every CSV carries the bag record time and, where the message has one, the header
stamp. The pipeline uses header stamps. If a sensor stamps with its own clock, its
header stamps are shifted by the median offset to the record clock. Periodic sources
still on record times are refit to the sensor period, allowing for dropped frames
(`regularize_times` in `fuse_ekf.py`).

Record times are not suitable for computing rates. On `hallway_04_cmd` the LiDAR scans
arrive at a mean of 10 Hz, but successive record times are between 0.1 ms and 208 ms
apart (standard deviation 8.3 ms), against 0.6 ms for the header stamps. An earlier
version of the filter used rates derived from record times, and the LiDAR-only filter
ended 2.90 m from its start against 1.09 m for KISS-ICP. The final filter takes
LiDAR and camera rotation as heading changes (below), which do not depend on interval
length. It gives 1.17 m with the time refit and 1.16 m without.

The remaining offset of each source relative to the gyro is estimated by
cross-correlating yaw rates and removed before fusion. The same correlation checks the
sign convention: a source whose yaw rate runs opposite to the gyro is reported, and its
sign is flipped for fusion.

IMU samples are checked before use. A sample is unusable when its stamp is more than
0.5 s off the usual record-minus-stamp lag, when all three gyro axes read exactly zero,
or when it belongs to a run of 50 or more identical readings. `extract_signals.py`
reports the usable share; `fuse_ekf.py` uses only usable samples.

### Visual odometry

`visual_odometry.py` uses about 12 images per second (every second image of a 25 Hz
recording, every image of a 15 Hz one), downscaled to at most 960 px wide. It reads the
left image, or the ZED `rgb` stream (the same camera) when a bag has only that.

- Corners are detected on a 4 x 3 grid of a lightly blurred copy (Gaussian, 1.5 px)
  and must exceed an absolute strength (minimum eigenvalue 5e-5). Without the absolute
  floor, a blank or washed-out image yields corners made of pixel noise, which track
  with zero motion and are reported as a stop.
- Each corner is back-projected with the registered depth (3 x 3 median, 0.3 to 10 m,
  depth edges rejected).
- Corners are tracked frame to frame with pyramidal Lucas-Kanade flow and a
  forward-backward check. The pose relative to the keyframe is solved by PnP in
  RANSAC (2 px), then refined on the inliers.
- A new keyframe is taken after 1 m, 15 deg, or when inliers fall below 120 or 40 % of
  the keyframe's corners.
- If tracking against the keyframe fails, the previous frame serves as a temporary
  keyframe ("recovered"). If that also fails, the frame is "lost", the pose is held,
  and the motion in that interval is not recovered. The filter skips lost intervals.

The trajectory is expressed in a level frame. The vertical is the axis of the camera's
own rotations during turns, since a robot on a level floor turns about the vertical.
IMU gravity is a cross-check (a warning is printed above 5 deg of disagreement), and
the fallback when a run contains less than 90 deg of turning. No IMU, wheel or LiDAR
data enters the visual-only poses.

### Filter

`fuse_ekf.py` runs a planar extended Kalman filter.

- State: position x, y; heading; forward speed v; yaw rate w; gyro bias b about the
  vertical; gyro scale error s (held at zero by default); and one heading clone per
  LiDAR or camera source.
- Motion model: constant speed and yaw rate, with white acceleration noise of 1.0 m/s^2
  and white angular acceleration noise of 2.0 rad/s^2. The bias follows a random walk
  of 2e-4 rad/s per square root of a second. There is no sideways speed (skid-steer),
  so the track follows the robot's turning centre for sensors mounted on the centreline.
- Wheel: speed and yaw rate, as 0.2 s means.
- Gyro: (1 + s) w + b at full rate, projected on the vertical measured during the
  stationary start.
- LiDAR and camera: forward speed over each 0.2 s interval, and the heading change over
  the same interval. At the start of each interval the filter copies its heading (the
  clone); at the end it compares its own heading change with the sensor's. A heading
  change constrains the heading itself, so the sensor's long-term heading carries
  through even though the gyro, at 256 Hz, dominates the short-term yaw rate. An
  earlier version fed the LiDAR as a yaw rate. The gyro then outweighed it at every
  step, the LiDAR's heading reached the filter only through the bias estimate, and
  LiDAR-inertial closed at 3.22 m on `hallway_04_cmd` against 1.19 m for LiDAR alone in
  that version.
- For LiDAR and visual odometry, the small yaw between the sensor frame and the
  direction of travel is estimated on straight segments, where the sideways speed must
  be zero.

| Source | Measurement | Standard deviation |
|---|---|---|
| Wheel | v, w (0.2 s means) | v: 0.03 + 0.05 abs(v) + 0.10 abs(w) m/s; w: 0.02 + 0.30 abs(w) rad/s (skid-steer slip grows with turn rate) |
| Gyro | (1 + s) w + b (full rate) | larger of 0.01 rad/s and 3 x the stationary standard deviation |
| LiDAR | v (0.2 s); heading change (0.2 s) | v: larger of 0.02 m/s and 1.5 x stationary deviation, + 0.05 abs(v); heading change: 0.2 s x (larger of 0.01 rad/s and 1.5 x stationary yaw-rate deviation), combined with 1 % of the turn |
| Visual | v (0.2 s); heading change (0.2 s) | as LiDAR, with floors of 0.03 m/s and 0.015 rad/s and 2 % of the turn |

These values were set from the sensor physics and the stationary data before any
evaluation. They have not been tuned on the metrics below.

Every measurement passes a chi-square innovation gate at 99 % (6.63 for one degree of
freedom, 9.21 for two). The share of each sensor's measurements rejected in a given
configuration is reported. In the `all` configuration this is the consistency check
between dissimilar sensors, and `benchmark_gating.png` shows where it fires.

Every run also reports the gyro's rotation in sustained turns as a percentage of each
other sensor's rotation. A scale error appears as the same ratio in every turn. The
filter can estimate the scale online (`--estimate-scale`). This is off by default:
on `hallway_04_cmd` it converged to +1.2 % against KISS-ICP, while the sustained-turn
ratio shows the gyro reading 1.1 % low. KISS-ICP without motion compensation is not an
unbiased rotation reference over short intervals. A scale measured elsewhere, for
example from in-place rotations through a known angle, can be applied with
`--gyro-scale`.

### Metrics

No ground truth is available. The metrics use what each run guarantees.

- Path length: positions sampled once per second, to suppress scan-to-scan jitter.
- Closure error: distance between start and end positions for a run that ended on its
  start mark. Reported in metres and as a percentage of a reference path length (the
  KISS-ICP path), the same denominator for every estimate.
- Heading reversal: angle between the principal directions (PCA) of the outbound and
  return legs of one straight corridor. True value 180 deg. For `hallway_04_cmd` the
  legs are 45 to 74 s and 146 to 192 s after the first LiDAR scan.
- Known distance: straight-line start-to-end displacement against a taped distance
  (`straight_5m`: 5.00 m).
- Heading change: total rotation from start to end, for comparison between estimates.
- Gate rejections: as above.

Presets exist for `hallway_04_cmd` and `straight_5m`. For other runs pass `--loop`,
`--outbound START END --return START END`, or `--known-distance METRES` to
`evaluate.py`.

## Data problems found

On `hallway_04_cmd`:

- ZED X IMU stream. Until 143.5 s the IMU stamps lag the recording by 10 to 40 ms and
  the gyro matches the wheel yaw rate (correlation 0.95 to 0.98 at zero lag). From
  143.5 s, messages keep arriving at the same rate (about 197 per second, so the
  recorder kept up), but their stamps advance at about a third of real time and the
  gyro reads close to zero while the robot turns. From about 171 s all three gyro axes
  read exactly zero (8,436 samples). 67 % of the IMU samples are usable. The
  configurations that include the gyro run on their other sensors after 143.5 s. The
  ZED SDK's own visual-inertial odometry probably lost the same IMU input; this is not
  verified.
- Gyro scale. In sustained turns the gyro measures 98.9 % of the KISS-ICP rotation. In
  the three large turns of the run the ratio is 0.986, 0.987 and 0.980. KISS-ICP's
  heading reversal of 179.7 deg indicates its rotation is the accurate one.
- LiDAR position. `tf_static` places the LiDAR 0.508 m ahead of `base_link`. The run
  ended 181 deg from its starting heading, so the KISS-ICP closure of 1.09 m is
  measured at the LiDAR. At the robot centre it is 1.22 m.
- ZED SDK odometry. The recorded `/zed/zed_node/odom` contains two discontinuities:
  29.4 m within one 33 ms step at 77.5 s, and 27.4 m with a 70 deg heading step at
  122.4 s. These two steps account for its 184 m path (KISS-ICP: 126 m) and its 52.75 m
  closure error. With the two steps removed, its closure error is 5.28 m (4.2 %) and its
  heading reversal 172.5 deg. The cause (tracking reset or relocalisation inside the
  SDK) is not determined.

On `straight_5m`, the robot was driven to a mark 5.00 m from the start; the stopping
point was not measured. The distance estimates disagree: wheel 5.33 m, RGB-D visual
odometry 5.07 m, KISS-ICP 4.67 m, ZED SDK 4.65 m. KISS-ICP and the ZED SDK are
independent and agree within 2 cm, which suggests the robot stopped short of the mark,
but the run cannot decide between the sensors. Closure error and heading reversal do
not detect a uniform scale error, so distance scale is an open question until a test
with a measured stopping point is done. The IMU was usable for the whole 50 s of this
run.

## Results

`hallway_04_cmd` (closed loop, 213.7 s, 125.6 m; gyro usable for the first 143.5 s):

| Estimate | Closure error | Heading reversal |
|---|---|---|
| Wheel (driver odometry) | 36.17 m (28.8 %) | 129.7 deg |
| LiDAR (KISS-ICP) | 1.09 m (0.87 %) | 179.7 deg |
| Visual (RGB-D VO) | 10.88 m (8.7 %) | 164.7 deg |
| Visual-inertial (ZED SDK) | 52.75 m (42.0 %) | 106.3 deg |
| Wheel (EKF) | 36.43 m (29.0 %) | 129.6 deg |
| Wheel-inertial (EKF) | 9.43 m (7.5 %) | 164.9 deg |
| Visual (EKF) | 2.85 m (2.3 %) | 178.8 deg |
| Visual-inertial (EKF) | 1.14 m (0.91 %) | 176.9 deg |
| LiDAR (EKF) | 1.16 m (0.92 %) | 179.4 deg |
| LiDAR-inertial (EKF) | 0.79 m (0.63 %) | 179.8 deg |
| Visual-LiDAR-inertial (EKF) | 1.15 m (0.91 %) | 178.3 deg |
| All sensors (EKF) | 0.69 m (0.55 %) | 178.8 deg |

`straight_5m` (49.9 s; driven to a mark 5.00 m from the start):

| Estimate | Start-to-end distance |
|---|---|
| Wheel (driver odometry and both wheel filters) | 5.33 m |
| Visual (RGB-D VO; visual and visual-inertial EKF) | 5.07 m; 5.06 m |
| All sensors (EKF) | 4.99 m |
| Visual-LiDAR-inertial (EKF) | 4.89 m |
| LiDAR (KISS-ICP; LiDAR and LiDAR-inertial EKF) | 4.67 m; 4.65 m |
| Visual-inertial (ZED SDK) | 4.65 m |

Observations:

- The gyro reduces heading-driven drift: wheel 29.0 % to 7.5 % with the gyro present
  for 67 % of the run, and visual 2.3 % to 0.91 %. Wheel-inertial stays within 3 deg of
  KISS-ICP until the gyro stops, then drifts to +22 deg by 200 s.
- LiDAR is the strongest single sensor (0.92 %). The best five configurations lie
  between 0.55 % and 0.92 %; one run does not separate them.
- The native RGB-D visual odometry (8.7 %) holds its pose through 21 lost frames, all
  during turns (85 s and 100 to 101 s). The filter bridges those gaps with its motion
  model, which is why the same data gives 2.3 % in the `visual` configuration.
- In the `all` filter, 64 of the 66 LiDAR rejections occur while the gyro is valid,
  mostly heading changes during fast motion. This is consistent with KISS-ICP running
  without motion compensation.
- On the straight legs of `hallway_04_cmd`, visual odometry measures 1.6 to 3.2 % more
  distance than KISS-ICP and the wheels about 8 % more; on `straight_5m` the spread is
  wider (above). The 4.99 m of the `all` filter on `straight_5m` is an average of
  opposing scale errors, not evidence that it is correct.

## Verification

### Synthetic bag with ground truth

`tests/make_synthetic_bag.py` writes a rosbag2 with the topics of `hallway_04_cmd` for
a 16 m out-and-back corridor run with two in-place 180 deg turns. Images and depth are
ray-cast, and the errors are known. Results with the final scripts:

| Check | Injected | Recovered |
|---|---|---|
| Camera tilt | 8.0 deg | 8.0 deg (camera's own turning axis; IMU gravity agrees within 0.1 deg) |
| Gyro bias about the vertical | 0.00439 rad/s | 0.00435 rad/s |
| LiDAR record-time lag | 80 ms | 82 ms (fallback path without header stamps) |
| Visual odometry vs truth | | mean position error 0.04 m, heading error 0.3 deg mean |
| Wheel yaw rate +35 % (slip) | | wheel-only closure 8.90 m; with the gyro 0.19 m |
| Gyro scale -1.3 % | | sustained-turn ratio 98.6 % (LiDAR), 98.8 % (camera); with `--estimate-scale`, -1.33 % |
| 2 s camera blackout | | 22 frames marked lost; native VO closure 1.32 m; visual EKF 0.01 m |
| Wheel yaw rate sign inverted | | detected, flipped, warning printed |
| IMU file with stale, zero and frozen samples | | flagged and excluded; tested on the real `hallway_04_cmd` IMU data |

### Real data

On `hallway_04_cmd`, the single-sensor filters reproduce the native estimators, which
checks the conversion to speed and heading change (numbers from the development run on
the uploaded CSVs; the laptop run above differs by at most 0.01 m):

| Estimate | Closure error | Heading reversal | Final heading |
|---|---|---|---|
| Wheel (driver odometry) | 36.17 m (28.8 %) | 129.7 deg | -121.3 deg |
| Wheel (EKF) | 36.43 m (29.0 %) | 129.6 deg | -121.3 deg |
| LiDAR (KISS-ICP), at the LiDAR | 1.09 m (0.87 %) | 179.7 deg | -181.3 deg |
| LiDAR (KISS-ICP), at the robot centre | 1.22 m | | |
| LiDAR (EKF) | 1.17 m (0.93 %) | 179.4 deg | -181.3 deg |

Percentages are of the 125.6 m KISS-ICP path.

## Limitations

- No ground truth. KISS-ICP serves as the plotting reference only.
- One run per condition. Differences of a few tenths of a metre need repeated runs
  before they are treated as real.
- The inertial results on `hallway_04_cmd` cover the gyro for 67 % of the run only.
- The fusion is loosely coupled. Tightly coupled estimators (FAST-LIO2, VINS-Fusion,
  ORB-SLAM3, FAST-LIVO2) are the next baselines.
- KISS-ICP runs without motion compensation, because the RoboSense driver does not
  publish per-point timestamps.
- Native tracks follow the sensor; EKF tracks follow the turning centre.
- Visual odometry fails explicitly (lost frames) on texture-poor views rather than
  reporting false motion. A high lost fraction in `vo_summary.txt` is a property of
  the scene and the method, not a fault of the run.
- The ZED SDK estimator is closed source; its settings are those used on the robot
  during recording.

## Next steps

1. Distance tests with a measured stopping point: stop near a mark, then tape the
   actual start-to-end distance to a fixed point on the robot. Three runs or more.
2. Find the cause of the ZED X IMU dropout and confirm a full-length IMU stream on a
   new recording (check `extract_summary.txt` right after recording).
3. Repeated closed loops, so each configuration has a spread rather than a single value.
4. Calibrate the gyro scale with in-place rotations through a known angle.
5. Re-record with per-point LiDAR timestamps (XYZIRT) to enable deskewing and FAST-LIO2.
6. Tightly coupled baselines on the same bags.
7. Additional sensors: thermal camera, event camera, UWB and GPS (outdoor runs). UWB
   ranges and GPS positions need their own measurement models in the filter. The speed
   and heading-change interface covers the cameras.
