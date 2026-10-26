# Rosbags

Rosbags recorded on the SCOUT2 rover. Maintainer: Vraj Damarla.

Rosbags are too large for Git. They are stored on the SRGE Lab OneDrive, in **Kilic, Cagri - Rover Team > Vraj Damarla > SCOUT2 > rosbags**. Lab members can request access from the lab lead.

## Recorded bags

| Bag | Date | Duration | Contents |
| :---- | :---- | :---- | :---- |
| `hallway_01` | 9/29 | 168 s | Hallway P route. LiDAR, wheel odometry, and transforms only. The ZED was not running. |
| `hallway_02` | October | | Hallway route. Details to be added from `ros2 bag info`. |
| `hallway_02_cmd` | 10/14 | 532 s | Hallway route with an extra out-and-back leg and a turnaround, Xbox controller at about 0.4 m/s. LiDAR, both ZED images, IMU, ZED and wheel odometry, `/joy`, `/mux/cmd_vel`. No depth. |
| `hallway_03` | October | 95 s | Hallway P route, AgileX remote at close to top speed (about 1.4 m/s). |
| `stationary_*` (two bags) | October | | Robot stationary. Details to be added from `ros2 bag info`. |
| `straight_5m` | Early October | | Tape-measured 5 m straight line, for measuring wheel odometry error. |

None of these runs had a measured end position. The next run (`hallway_04_cmd`) uses a taped start and end mark.

## Results

`scripts/kiss_vs_wheel.py` compares KISS-ICP LiDAR odometry with wheel odometry on a bag. Both hallway runs return along the corridor they started in, so a consistent estimate draws the return pass over the outbound pass.

| Bag | Return pass vs. outbound pass, KISS-ICP | Return pass vs. outbound pass, wheel odometry | Final heading difference |
| :---- | :---- | :---- | :---- |
| `hallway_02_cmd` (slow) | 0.4 m | 18.7 m | 51 deg |
| `hallway_03` (fast) | 3.6 m | 0.9 m | 2 deg |

Wheel odometry lost heading on the slow run, which includes a tight turnaround. KISS-ICP degraded on the fast run, likely because the LiDAR driver does not publish per-point timestamps, so scans cannot be corrected for motion. Neither run had ground truth.

Each bag is a folder containing a `.db3` file and a `metadata.yaml`. Both are
required.

## Recording a bag

On the Jetson, inside tmux, with the ZED running ([start-up guide](startup_guide.md), section 9.5):

```
~/record_bag.sh NAME [SECONDS]
```

With `SECONDS`, recording stops automatically. Without it, stop with `Ctrl+C`.

A name ending in `_cmd` also records `/joy` and `/mux/cmd_vel`. Use this for
runs driven with the Xbox controller.

The script ([`scripts/record_bag.sh`](../scripts/record_bag.sh)) has not yet been verified on the robot. Until it has, use the manual commands in the start-up guide, section 9.10. It must be copied
to the Jetson before first use. From PowerShell on a laptop, in the repository
folder:

```
scp scripts\record_bag.sh indro@192.168.42.42:~/
```

Then on the Jetson:

```
chmod +x ~/record_bag.sh
```

The full procedure is in the [start-up guide](startup_guide.md), section 9.10.

## Safety

Bags ending in `_cmd` contain drive commands. Do not run `ros2 bag play` on
the robot while it is powered. Play bags only on another computer.

## Credentials

Credentials are not stored in this repository. They are in Appendix A of the
internal Scout 2.0 manual on the SRGE Lab OneDrive.
