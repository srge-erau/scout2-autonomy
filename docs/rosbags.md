# Rosbags

Rosbags recorded on the SCOUT2 rover. Maintainer: Vraj Damarla.

Rosbags are too large for Git. They are stored on OneDrive: [PASTE ONEDRIVE LINK]

## Recorded bags

| Bag | Date | Duration | Contents |
| :---- | :---- | :---- | :---- |
| `hallway_01` | 9/29 | 168 s | Hallway P route. LiDAR, wheel odometry, and transforms only. The ZED was not running. |
| `straight_5m` | Early October | | Tape-measured 5 m straight line, for measuring wheel odometry error. |
| `hallway_02_cmd` | 10/14 | 532 s | Hallway P route, Xbox controller. LiDAR, both ZED images, IMU, ZED and wheel odometry, `/joy`, `/mux/cmd_vel`. No depth. |

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

The script ([`scripts/record_bag.sh`](../scripts/record_bag.sh)) must be copied
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
