# Scout 2.0 / InDro Commander: ERAU

## Operations & Reference Manual

**Platform:** AgileX Scout 2.0 with InDro Commander payload
**Owner:** ERAU, SRGE Lab
**Status:** Working draft. Consolidated from the (now abandoned) InDro documentation site and the ERAU startup/teleoperation log.
**Last updated:** 10/23/26

### About this document

The vendor documentation lived on a website that is no longer maintained. This manual is the replacement. It combines that site's content with ERAU's commissioning notes and is maintained by the SRGE Lab.

Two conventions used throughout:

* `[IMAGE: description]` marks a place where a figure belongs. See the Appendix for the full list.
* `[VERIFY]` marks a claim inherited from the vendor site that ERAU has not independently confirmed, or where two sources disagree. See Section 11.

Credentials are not included in this document. They are in Appendix A of the internal Scout 2.0 manual on the SRGE Lab OneDrive.

### Revision note, 10/23/26

This revision adds findings from ERAU sessions between mid-September and October 23. Summary of what changed:

* **Added:** Section 0, a quick-start procedure from power-on to a recorded rosbag and a controlled shutdown, intended for a new lab member to follow without assistance.
* **Added:** recording rosbags (9.10), copying them off the robot (9.11), and shutting down (9.12).
* **Corrected:** the robot's own Wi-Fi, `InDro Robotics_ERAU`, can be used to connect. The previous revision stated otherwise.
* **Added:** how to keep control when driving out of the lab (5.5).
* **Corrected:** the container list (3.3), the host operating system (3.1), and the ZED topic names (9.5). The vendor documentation was incorrect on all three.
* **Confirmed:** ZeroTier network ID `3850b435d4b223c6` works. RViz only works over ZeroTier.
* **Added:** a u-blox GNSS receiver and antenna are installed, with no driver running.
* **Added:** the ZED launch file had been accidentally edited, which stopped the camera from starting. The recovery is in 11.3.
* **Updated:** the arm investigation (9.7.4). Arm work is on hold, per the lab lead.

**Previous revision, 9/14/26:** merged ERAU's commissioning notes into the vendor documentation, added the NUC's addresses, documented the Jetson hang failure mode (11.1), and documented the arm's lost zero reference and its recovery (9.7.3).

---

## 0. Quick start

Follow this from the top for a normal session. Each step points to the section with more detail. Commands run in PowerShell on your laptop unless the step says **on the robot**.

### 0.1 What you need

* A laptop. For Ethernet, it needs an Ethernet port or a USB Ethernet adapter.
* An Ethernet cable with an intact retaining clip on both plugs (11.3)
* The robot credentials, from Appendix A of the internal Scout 2.0 manual on the SRGE Lab OneDrive
* The AgileX remote, as a backup way to drive and stop the robot
* Optional: an Xbox controller with a USB data cable

### 0.2 Power on

1. Check both E-Stops are released. Twist to release a latched one. See 1.2.
2. Check the arm is in a low, stable pose. See 1.3.
3. Turn the ignition key at the rear, then press the power button next to the voltage display.
4. Wait 3 minutes for the router and the Jetson to boot.

### 0.3 Connect your laptop

Pick one. Details are in Sections 5 and 6.

| Method | Robot address | When to use it |
| :---- | :---- | :---- |
| Ethernet, into the back-panel port | `192.168.42.42` | In the lab. Most reliable, and fastest for copying bags. |
| The robot's Wi-Fi, `InDro Robotics_ERAU` | `192.168.42.42` | Walking with the robot. Your laptop has no internet while joined. |
| ZeroTier | `10.13.46.42` | Working remotely, or using RViz. Needs the robot on lab Wi-Fi. |

Check the connection:

```
ping 192.168.42.42
```

If there are no replies, see 11.1.

### 0.4 Open a terminal on the robot

```
ssh indro@192.168.42.42
```

The first time, type `yes`. Then enter the Jetson password (see the internal manual on OneDrive). Nothing appears while you type. When the prompt reads `indro@indro-commander:~$`, every command you type runs **on the robot**. Type `exit` to return to your laptop.

### 0.5 Check the clock

**On the robot:**

```
timedatectl
```

Do not record anything until it says `System clock synchronized: yes`. The Jetson starts at 1 January 1970 on every cold boot and only gets the right time once it reaches the internet. Bags recorded before then carry wrong timestamps. See Known Issue 14.

### 0.6 Start the ZED camera

The ZED does not start on its own. Start it inside tmux, so it keeps running if your connection drops.

**On the robot:**

```
tmux new -s zed
```

```
front
```

Wait for log lines ending with an initial camera pose. Then detach, leaving it running: press `Ctrl+B`, release, then press `D`.

Check it is publishing:

```
ros2 topic list | grep zed_node/imu
```

If `front` fails with a `SyntaxError`, see 11.3.

### 0.7 Open the dashboard

In a browser:

```
http://192.168.42.42/console
```

Type `http`, not `https`. Check the status bar (9.1). Clear the area around the robot and arm, then press **START**.

### 0.8 Drive

* **AgileX remote.** Talks straight to the chassis over its own radio. It keeps working if the Jetson or the Wi-Fi fails. Its commands are not recorded in ROS.
* **Xbox controller.** Plug it into your laptop, open the **GAMEPAD** tab, and press a button. Hold **LB** and use the left stick. Commands go out on `/mux/cmd_vel` and can be recorded.

Do a short test move in the lab first, and confirm that releasing LB stops the robot. See 9.6.

### 0.9 Record a rosbag

Follow 9.10. Always record inside tmux, and wait for `All requested topics are subscribed` before moving the robot.

### 0.10 Copy the data off, then shut down

Copy each bag to your laptop and OneDrive (9.11). Then, **on the robot:**

```
sudo shutdown now
```

Wait for the Jetson to go dark, then turn the key off. Never cut power with the Jetson running. See 9.12.

---

## 1. Safety

### 1.1 Charging

Do not charge the robot overnight or for any extended period. The charger will begin behaving abnormally. It blinks red and stops charging the robot entirely. Unplug it once the pack is full.

### 1.2 Emergency stop

The E-Stop immediately halts all robot motion and operations.

| Item | Detail |
| :---- | :---- |
| Function | Instantly stops all robot movements and operations |
| Location | One on each side of the base platform |
| Use | Press to stop the robot in an emergency or when unsafe behaviour is observed |
| Reset | Rotate the same button to release it and resume normal operation |

`[IMAGE: E-Stop button location on the base platform]`

Physically confirm both E-Stops release cleanly before any drive test. A latched E-Stop is a common reason the chassis accepts commands but the wheels do not move.

### 1.3 Arm collapse on power loss

**The arm joints have no brakes.** If the robot loses power while the arm is raised, the arm falls under its own weight. This has two consequences:

1. Anything underneath the arm, including hands, cables, and the chassis itself, is in the path.
2. The controller loses its zero reference. The arm will afterwards drive to a "home" position that does not match physical reality, and recovering it requires a manual per-joint re-teach. See Section 9.7.3.

Before powering down, for any reason including an E-Stop press, **bring the arm to a low, stable, supported pose.** This step is part of the shutdown procedure.

This has already happened once on this unit. See the Day 3 log in Section 12.

### 1.4 Rosbag replay

**Never run `ros2 bag play` on the robot while it is powered.** A bag that contains drive commands (`/mux/cmd_vel`) will re-send them, and the robot will execute them without feedback. Play bags only on another computer. Bags containing commands are named with `_cmd` (9.10).

---

## 2. System overview

### 2.1 What the InDro Commander is

The InDro Commander is a payload stack mounted on an AgileX Scout 2.0 base platform. The Scout provides the chassis, drivetrain, battery, and CAN interface. The Commander box adds compute, networking, sensors, and the manipulator. The two communicate over CAN.

`[IMAGE: Full robot, exterior view]`

### 2.2 Hardware components

**Base platform**

* AgileX Scout 2.0 (4WD skid-steer chassis, 24 V nominal battery)

**Compute**

* NVIDIA Jetson Orin NX (Engineering Reference Developer Kit), primary ROS compute. Hostname `indro-commander`. Confirmed 10/23 with `cat /proc/device-tree/model`. The vendor text said Orin AGX, which is incorrect.
* Intel NUC, secondary compute. Hostname `indro-commander-nuc`. Ubuntu 20.04 on x86_64, with no robot software installed (3.1).

**Networking**

* WG827 5G router (Wi-Fi and cellular WAN). It also broadcasts its own Wi-Fi, `InDro Robotics_ERAU` (5.5).
* Netgear GS105 5-port network switch (mounted beneath the router)

**Power**

* Power distribution block (12 V) and ground block
* 24 V to 12 V converter

**Sensors and manipulator**

* ZED X stereo camera (front), with a Stereolabs compute module in the electronics bay. It contains the robot's only IMU.
* Pin-hole camera (rear)
* RoboSense LiDAR
* u-blox GNSS receiver on USB, with a crossed-dipole antenna under a black radome on the chassis top. No driver is installed, so it publishes nothing. See Known Issue 15.
* InDro Manuvr arm with gripper. `[VERIFY: see Known Issue 8]`

**USB devices on the Jetson**, from `lsusb`:

| Device | USB ID | Role |
| :---- | :---- | :---- |
| Geschwister Schneider CAN adapter | `1d50:606f` | CAN link to the chassis |
| u-blox GNSS receiver | `1546:01a9` | GNSS, unused |
| Two FTDI serial bridges | `0403:6015` | `/dev/ttyUSB0`, `/dev/ttyUSB1`, purpose unknown |
| Intel AX210 | `8087:0032` | Bluetooth |

`/dev/ttyACM0` is also present.

`[IMAGE: Interior of the Commander box, showing Jetson, NUC, converter, power distributor, CAN-BUS, network switch and router]`
`[IMAGE: Exterior labelled view, showing arm, gripper, LiDAR, ZED X, GNSS antenna]`

### 2.3 Network addresses

Credentials are in Appendix A of the internal Scout 2.0 manual on the SRGE Lab OneDrive.

| Device | Local IP | ZeroTier IP |
| :---- | :---- | :---- |
| Jetson Orin NX (`indro-commander`) | 192.168.42.42 | 10.13.46.42 |
| Intel NUC (`indro-commander-nuc`) | 192.168.42.43 | 10.13.46.43 |
| WG827 router | 192.168.42.1 | 10.13.46.1 |
| LiDAR | 192.168.42.205 | none |
| Pin-hole camera | 192.168.42.201 `[VERIFY, see 2.5]` | none |
| Arm (per vendor table) | 192.168.42.110 `[VERIFY, see 9.7.4]` | none |

`192.168.42.x` is the robot's own subnet, reachable over Ethernet or the robot's own Wi-Fi. `10.13.46.x` is the same devices reached over ZeroTier from anywhere on the internet. The last number carries across: `.42` is the Jetson on both networks. Only the Jetson, NUC, and router have ZeroTier addresses. To reach the LiDAR, camera, or arm remotely, SSH into the Jetson first.

The NUC row was added by ERAU. It is not in the vendor table. The NUC is configured with a **static** address, not DHCP, set in NetworkManager under a connection profile named "42". Confirmed via `nm-connection-editor`.

### 2.4 Power and wiring

Battery nominal voltage is 24 V.

**Power path:**

Base battery (24 V) > 24 V to 12 V converter > 12 V distribution block > ground block > 24 V rail > Jetson Orin > NUC > Router > Network switch > Camera box > LiDAR box > Arm

**Data path:**

* **Ethernet:** Orin, NUC, router, switch, camera box, LiDAR box, and arm are all on the switch.
* **CAN bus:** CAN High and CAN Low run from the Scout chassis to the CAN-BUS breakout, which reaches the Orin over USB.

Anything on the power line requires a common ground. This is specified on the vendor wiring diagram. Check it first when a peripheral behaves erratically.

The arm's control box is powered through an **XT60 connector** at the arm base, separate from its Ethernet. Power cycling the arm alone means unplugging that connector for 10 to 15 seconds. The control box carries a green power LED and a blue status LED; solid green with blinking blue is the healthy state.

`[IMAGE: InDro Commander wiring diagram]`

### 2.5 Confirmed host inventory

A full scan of the robot subnet was run from the NUC during commissioning:

```
nmap -sn 192.168.42.0/24
```

Exactly six hosts responded:

| Address | Identified as | How |
| :---- | :---- | :---- |
| 192.168.42.1 | WG827 router | LuCI web interface |
| 192.168.42.42 | Jetson Orin NX | SSH, hostname |
| 192.168.42.43 | Intel NUC | SSH, hostname |
| 192.168.42.101 | Camera, ACTi | MAC prefix `00:0f:7c` |
| 192.168.42.110 | Arm | vendor address table |
| 192.168.42.205 | RoboSense LiDAR | vendor address table |

**The camera address does not match the vendor table.** The table lists the pin-hole camera at `.201`. The scan found a camera at `.101` and nothing at all at `.201`. The two most likely explanations are a transcription error in the vendor documentation (201 and 101 differ by one character) or a camera that was readdressed at some point. Until this is settled, use `.101`, the address that responds.

Run this scan when a device stops responding. It takes under a minute and distinguishes a device that is off the network from one at an undocumented address.

---

## 3. Software environment

### 3.1 Host vs. container: read this first

The vendor documentation says the Jetson host runs Ubuntu 20.04 with ROS 2 Foxy. What ERAU observed on this unit is different:

* **Jetson host.** `apt` installs Ubuntu 22.04 (jammy) arm64 packages, and ROS 2 Humble commands work directly in an SSH session. The ZED driver runs on the host from `~/colcon_ws`, started with the `front` command (9.5). Confirm the version with `lsb_release -a`.
* **InDro stack.** The dashboard, chassis driver, LiDAR driver, rosbridge, router interface, and Xbox teleop each run in their own Docker container (3.3).
* **NUC.** Ubuntu 20.04.6 on x86_64. Docker is not installed and nothing robot-related runs on it. Its intended role is undocumented. Its last login before ERAU's sessions was in March 2025.

**Effect on recording.** A message type defined only inside a container cannot be recorded from the host. `/scout_status` uses `interfaces/msg/ScoutStatus`, which exists only in the chassis driver's container, so `ros2 bag record` on the host skips it with an "unknown type" warning. Every other topic records normally.

Client laptops running ROS 2 should use Humble.

### 3.2 Stack summary

| Layer | What runs there |
| :---- | :---- |
| Jetson host | Ubuntu 22.04 arm64, ROS 2 Humble, ZED driver (`~/colcon_ws`), Docker engine, ZeroTier |
| Containers | InDro dashboard, chassis driver, LiDAR driver, rosbridge, router interface, Xbox teleop, camera streaming |
| NUC | Ubuntu 20.04 x86_64, no robot software |
| Transport | CAN (chassis, through a USB adapter), Ethernet (LiDAR, camera, compute), USB (GNSS, serial bridges) |
| Remote access | ZeroTier, or the robot's own Wi-Fi |

### 3.3 Containers on this unit

The vendor documentation lists six containers. Only one of them, `robosense`, exists on this unit under that name. Verified on the Jetson with:

```
docker ps -a
```

**Running:**

| Container | Image | Role |
| :---- | :---- | :---- |
| `indro_controller_react` | `indro_controller:react_build` | Dashboard front end |
| `indro-controller` | `indro_base:latest` | Dashboard back end |
| `docker_api` | `nginx_indro:latest` | Web server on port 80 |
| `rosbridge` | `indro_base:latest` | WebSocket bridge between the dashboard and ROS |
| `xbox_teleop` | `indro_base:latest` | Turns gamepad input into drive commands |
| `base_platform` | `indro_base:latest` | Chassis driver over CAN |
| `robosense` | `indro_base:latest` | LiDAR driver |
| `wg827_router` | `indro_base:latest` | Router status, published on `/router/modem_info` |
| `rtsp_server` | `bluenviron/mediamtx:latest` | Camera stream server |
| `rtsp-webrtc` | `ghcr.io/deepch/rtsptoweb:v2.4.3` | Camera stream to the browser. **In a continuous restart loop** (Known Issue 13). |
| `speedtest-tracker` | `lscr.io/linuxserver/speedtest-tracker:0.20.0` | Network speed monitor on port 8080 |

**Stopped:** `ros_1` and `test`, both exited about 18 months before inspection.

The vendor list's `Scout_mini`, `swiftnav_ros2`, and `oak_ros2` do not exist here. The chassis driver is `base_platform`.

`[IMAGE: docker ps output]`

---

## 4. Startup procedure

### 4.1 Pre-start checks

1. Confirm both E-Stops are released (rotate each one).
2. Confirm the arm is in a low, stable pose. See Section 1.3.
3. If using a monitor, plug the HDMI cable into the monitor before powering on.
4. Check the battery voltage readout near the rear panel. Nominal is 24 V.

### 4.2 Power on

Two things must happen, and they are separate:

1. **Ignition key.** Located at the rear of the rover, adjacent to the charging port. Turn it.
2. **Power button.** On the rear panel, next to the voltage display. Press it.

`[IMAGE: Rear panel showing power button and voltage display]`

The Scout 2.0 has a built-in 10-second delay after power-on while it prepares to become fully operational. The Commander payload takes longer. Allow 2 to 3 minutes for the router to bring up its network connections.

### 4.3 First-boot verification

Work through these in order. Each one depends on the previous.

1. Router reachable at `192.168.42.1`
2. Jetson reachable at `192.168.42.42`, or `10.13.46.42` over ZeroTier
3. Clock synchronised: `timedatectl` shows `System clock synchronized: yes` (Known Issue 14)
4. Containers up: `docker ps` shows eleven running. `rtsp-webrtc` showing "Restarting" is a known issue.
5. LiDAR publishing: `ros2 topic list | grep rslidar` shows `/rslidar_points`
6. ZED publishing, after starting it (9.5): `ros2 topic list | grep zed_node`
7. Chassis reporting healthy: `vehicle_state: 0` and `error_code: 0`
8. For ZeroTier, the router shows a connection to the lab hotspot (`SRGE_Local`)

If step 2 fails, go to 11.1 before troubleshooting the network. The Jetson has repeatedly been observed powered on but unresponsive, which presents the same symptoms as a cabling fault.

---

## 5. Network setup

Wi-Fi configuration is a **one-time setup** unless you are changing networks. Once configured, the robot reconnects on its own at every boot.

### 5.1 Reaching the router configuration page

1. Connect an Ethernet cable to the **Ethernet connector on the back of the InDro Commander.**
2. Set your laptop's Ethernet adapter to DHCP. If DHCP does not assign an address, set a static IP in the `192.168.42.xx` range.
3. Open a browser and type the router address directly into the address bar. Do not type it into a search box.

```
http://192.168.42.1
```

4. Log in with the router password (see the internal manual on OneDrive).

> **Troubleshooting: Firefox forces HTTPS.** Firefox may automatically upgrade `http://` to `https://` and then fail with "unable to connect". Edit the address bar and delete the `s` from `https://`. This applies to the dashboard at `10.13.46.42` as well.

`[IMAGE: Router login page]`

**Two cautions on the physical connection:**

**Use the back-panel port, not the internal switch.** The GS105 inside the electronics bay is on the same subnet, so it works electrically, but reaching into the bay risks dislodging sensor cabling and there may be no free port.

**Never plug the lab network into the robot's LAN.** Running a drop from a room access point into the back-panel port or the GS105 bridges ERAU's network into the robot's internal subnet. That puts two DHCP servers on one segment, the campus one and the robot's own, and devices with DHCP-assigned addresses may take a lease from the wrong one. If this has happened, verify every host in the Section 2.5 table still answers before continuing, and reboot anything that does not. The robot's uplink to the lab network belongs on the router's WAN side, configured through Hotspot Manager, not patched into the LAN.

> **Ceiling-mounted access points may carry PoE.** Some outdoor access points, including the WAVLINK unit above the SRGE bench, are powered over Ethernet. Never plug a laptop or the GS105 into the output side of a passive PoE injector. Passive injectors put voltage on the line unconditionally and can damage a non-PoE port.

### 5.2 Connecting to Wi-Fi

1. Navigate to **WIFI Hotspot > Hotspot Manager.**
2. Select the network and enter its password.
3. Click **Connect to Hotspot** at the top of the page and wait.
4. Verify the hotspot shows as connected. At ERAU this should read `SRGE_Local`.
5. On success the router shows a **yellow light.**

If the page reads "checking for connection", the router was recently booted. Give it a few seconds.

`[IMAGE: Hotspot Manager showing SRGE_Local connected]`

> **The robot also broadcasts its own Wi-Fi, `InDro Robotics_ERAU`.** Joining it puts your laptop on `192.168.42.x`, the same as Ethernet. An earlier revision of this manual stated that it did not work. It was confirmed working in September. See 5.5.

### 5.3 Cellular fallback

If no Wi-Fi is available, the robot can use a cellular network.

1. Insert a SIM card into the SIM card extender.
2. Power cycle the robot.

> **Reboot the modem if you hot-plug the SIM.** Inserting a SIM without a reboot leaves the modem in an unusable state.

### 5.4 Behaviour after setup

1. On power-up, the robot searches for saved Wi-Fi networks and connects.
2. If a SIM is present it also brings up cellular, giving multiple WAN interfaces.
3. Wi-Fi always takes priority over cellular.
4. Allow 2 to 3 minutes for the router to establish an internet connection over either path.
5. Once the robot has internet, you no longer need the Ethernet cable to reach the router config page. Use the ZeroTier address `10.13.46.1` instead.

> **Effect on remote access.** ZeroTier depends on the router's WAN uplink. If the lab Wi-Fi drops, the robot's ZeroTier addresses go with it while the local `192.168.42.x` subnet stays up. When `10.13.46.42` is unreachable, test `192.168.42.42` from a machine on the robot's own subnet before concluding the robot is down. From off-site, the two failures present identically but have unrelated causes.

### 5.5 The robot's Wi-Fi, and driving out of the lab

The router broadcasts its own network, `InDro Robotics_ERAU` (password in the internal manual on OneDrive), while also connected to the lab Wi-Fi. Joining it puts your laptop directly on the robot's subnet. While on it:

* Use `192.168.42.42` for everything.
* ZeroTier does not work, and your laptop has no internet.
* RViz does not work, since it needs ZeroTier (8.1).

**Driving out of the lab.** When the robot was driven beyond the lab Wi-Fi's range, Xbox control through the dashboard dropped out. The likely cause is the router trying to keep its lab Wi-Fi connection as the signal fades, which disrupts its own hotspot. This was not confirmed. Before a hallway run:

1. Check the clock on the robot (`timedatectl`) first, since the next step cuts the robot's internet.
2. Open `http://192.168.42.1`, go to **WIFI Hotspot > Hotspot Manager**, and disconnect from SRGE_Local.
3. Join your laptop to `InDro Robotics_ERAU`. In Windows, turn off **Connect automatically** for the lab networks (Settings > Network & Internet > Wi-Fi > Manage known networks), so the laptop does not jump back to them.
4. Disconnect ZeroTier on your laptop.
5. Use the dashboard at `http://192.168.42.42/console`, never the `10.13.46.x` address.
6. Keep a continuous ping running while you walk, to watch the link: `ping -t 192.168.42.1`. `Ctrl+C` stops it.
7. After the run, reconnect the router to SRGE_Local, so ZeroTier and the clock work again.

The AgileX remote does not use Wi-Fi at all. Carry it on every run outside the lab.

---

## 6. ZeroTier

### 6.1 Purpose

ZeroTier creates a virtual network that makes devices behave as if they were on the same LAN, regardless of where they physically are. Functionally it is a VPN, but without the configuration overhead.

For this robot it provides three things:

* **Remote access.** Control the robot from anywhere, even when your laptop and the robot are on completely different networks.
* **No network plumbing.** No port forwarding, no firewall rules.
* **Encrypted transport.** Traffic between your machine and the robot is encrypted.

Once joined, the robot is reachable at `10.13.46.42` (Jetson) and `10.13.46.1` (router) as though it were plugged into your laptop.

### 6.2 Network ID

```
3850b435d4b223c6
```

> `[VERIFY]` The vendor site shows this ID in its callout box, but a terminal capture on the same page shows `3850b435d4bbb3f9`. One of these belongs to a different robot. Confirm against `sudo zerotier-cli listnetworks` on a machine already joined to ours before relying on either.

**Partial evidence gathered 9/14/26.** A laptop joined to `3850b435d4b223c6` progressed to status `ACCESS_DENIED`. A nonexistent network ID leaves a client at `REQUESTING_CONFIGURATION` indefinitely, so a controller does exist at that ID and did respond. This confirms that the network exists, not that it is this robot's network. The definitive check is still to run `listnetworks` on the Jetson or the NUC and compare.

A second observation points the other way. On Linux, the ZeroTier interface name is derived from the network ID, so every Linux host on the same network shows the same name. Both robot machines show `ztbwcamfqs`. The vendor page's worked example shows `ztbwca7brn`. Those are different networks. The vendor example was copied from some other deployment, which is consistent with the ID conflict above and reduces confidence in the vendor page.

**Confirmed working, late September 2026.** A laptop joined to `3850b435d4b223c6` was authorised by InDro support and reached the Jetson at `10.13.46.42`, with RViz working over it. That settles the ID conflict above in favour of `3850b435d4b223c6`.

Authorisation is manual. InDro must approve your device before it can reach anything. Email **support@indrorobotics.com** with your ZeroTier address to get authorised.

> The vendor page renders this address as `support@indrorobotics` with no domain ending, which is not a valid address. The company's mail domain is `indrorobotics.com`, even though their public website is at `indrorobotics.ca`. If `support@` bounces, `info@indrorobotics.com` is the general contact.

### 6.3 Installation on Windows

1. Download and install **ZeroTier One** from the official site.
2. Launch ZeroTier. There is no main window, it runs in the background.
3. Click the `^` arrow in the bottom-right of the taskbar to reveal hidden icons, then click the ZeroTier icon.

`[IMAGE: ZeroTier icon in the Windows system tray]`

4. Click **Join New Network.**
5. Enter the 16-digit Network ID and click **Join.**

`[IMAGE: Join ZeroTier Network dialog]`

6. Note your own address (shown as **My Address** in the ZeroTier menu) and send it to InDro support for authorisation.
7. Once authorised, verify by pinging the robot's ZeroTier IP.

**Expected status progression.** The menu shows a Status field for the joined network. It moves through:

| Status | Meaning |
| :---- | :---- |
| `REQUESTING_CONFIGURATION` | The join request has been sent and is awaiting a reply |
| `ACCESS_DENIED` | The controller received the request and is holding it pending manual approval. **This is normal and is not an error.** |
| `OK` | Authorised. A `10.13.46.x` address is now assigned |

Nothing on your end changes between `ACCESS_DENIED` and `OK`. Leave ZeroTier running and wait for approval.

**Before emailing InDro, ask locally.** Authorisation is a single checkbox in ZeroTier Central for whoever administers the network. If the SRGE Lab holds those credentials, a lab member can approve a new node in seconds. The InDro support queue is in Canada and is the slower path.

### 6.4 Installation on Ubuntu

Install the CycloneDDS RMW for Humble:

```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Install ZeroTier:

```
curl -s https://install.zerotier.com | sudo bash
```

Join the network:

```
sudo zerotier-cli join <network_id>
```

Check your node status and address:

```
sudo zerotier-cli info
```

A healthy response looks like `200 info <address> <version> ONLINE`.

List the networks you have joined:

```
sudo zerotier-cli listnetworks
```

The output columns are `<nwid> <name> <mac> <status> <type> <dev> <ZT assigned ips>`. Status should read `OK`.

`[IMAGE: zerotier-cli listnetworks output]`

---

## 7. CycloneDDS

CycloneDDS is an implementation of the DDS (Data Distribution Service) specification, the publish/subscribe middleware ROS 2 uses underneath. Configuring it to bind to the ZeroTier interface is what lets ROS topics cross between your laptop and the robot over the internet.

> Tested with Humble on the client machine and Humble in Docker on the InDro Commander computer. Mixing distros across that boundary is not supported.

### 7.1 Finding your ZeroTier interface name

Joining a ZeroTier network creates a new network interface on your machine. You need its name for the CycloneDDS config.

```
sudo zerotier-cli listnetworks
```

The interface name appears in the `<dev>` column and begins with `zt`. On this robot's network it is `ztbwcamfqs`. Both the Jetson and the NUC show that name, as expected, since the name derives from the network ID.

Alternatively:

```
ifconfig
```

If `ifconfig` is not available:

```
sudo apt install net-tools
```

Any interface beginning with `z` is a ZeroTier interface. If you have joined more than one network, more than one will appear. Match the name against the `listnetworks` output.

`[IMAGE: ifconfig output showing the zt interface]`

### 7.2 Writing the CycloneDDS configuration

> `[TODO: MISSING SOURCE]` The vendor page continues past the interface-name step with the actual `cyclonedds.xml` configuration, and the `CYCLONEDDS_URI` and `RMW_IMPLEMENTATION` environment variables needed to make ROS 2 use it. That section was not captured before the site was abandoned.
>
> A `cyclonedds.xml` exists inside each container directory under `~/docker/` on the robot (see Section 10.3). Pulling one of those and adapting it for the client machine is the most likely path to reconstructing this section. Until then, treat remote ROS topic access as unconfigured.

**Note.** Because this section has not been reconstructed, a native ROS 2 install on a client laptop is currently the most difficult way to control the robot. DDS discovery relies on multicast, which does not reliably cross a VPN or campus wireless. Sections 9.6, 9.8, and 9.9 do not depend on DDS discovery and should be used until 7.2 is rebuilt.

---

## 8. Connecting to the onboard computer

There are three ways in. Pick based on what you are doing.

| Method | Use it when |
| :---- | :---- |
| InDro Controller UI | Driving, viewing sensors, normal operation |
| SSH | Development, debugging, anything container-related |
| HDMI plus keyboard and mouse | Network is broken and you need a local console |

### 8.1 InDro Controller UI

The InDro Controller is a browser-based interface with a gamepad tab for remote control, a camera tab for live streaming, and a built-in terminal.

Two addresses depending on how you are connected:

| Address | When to use |
| :---- | :---- |
| `http://192.168.42.42` | Connected by Ethernet or the robot's onboard Wi-Fi |
| `http://10.13.46.42` | Robot is on the internet and your machine is on the ZeroTier network |

The console view is at `/console` and the map view at `/map`.

> **RViz only works over ZeroTier.** The RViz panel on the Missions page connects to `10.13.46.42` directly. Over Ethernet or the robot's Wi-Fi, the rest of the dashboard loads but RViz does not. Use ZeroTier for anything involving RViz, even in the lab.

### 8.2 SSH

1. Power on the robot and confirm it is up.
2. Open a terminal. On Linux and macOS use the system terminal. On Windows use PowerShell or Git Bash.
3. Connect.

If you are on the robot's own Wi-Fi or wired to it:

```
ssh indro@192.168.42.42
```

If the robot is on the lab's Wi-Fi and you are on ZeroTier:

```
ssh indro@10.13.46.42
```

4. On first connection, accept the host key by typing `yes`. The full word is required, `y` alone is rejected.
5. Enter the Jetson password when prompted (see the internal manual on OneDrive). No characters appear on screen while typing. That is expected.

Once connected, the prompt changes to `indro@indro-commander:~$`. Every command typed after that point runs **on the robot**, not on your own machine. Type `exit` to return.

VS Code's Remote-SSH extension works over either address and is recommended for anything beyond a quick check.

### 8.3 HDMI cable

For direct local access with a monitor, keyboard, and mouse:

1. Power on the robot.
2. Plug HDMI from the Jetson into a monitor or TV.
3. Connect a mouse and keyboard to the Jetson's USB ports.
4. Once the robot is fully powered on, the desktop appears on the monitor.
5. Log in with the Jetson password (see the internal manual on OneDrive).

This is the fallback when the network is misconfigured. It is the only method that does not depend on networking working.

> **Check the hostname in the prompt before using a local console session.** Both onboard machines can have a monitor attached, and `indro-commander` and `indro-commander-nuc` look identical at a glance. A session on the NUC will show `192.168.42.43` and `10.13.46.43`, not `.42`. During commissioning, the wrong machine was diagnosed on more than one occasion.

---

## 9. Operating the robot

Everything in this section assumes you are connected to ZeroTier and have the InDro UI open at `10.13.46.42`, or at `192.168.42.42` locally.

### 9.1 Accessing the InDro UI

1. Open ZeroTier One and confirm you are connected to the correct network.
2. Open any browser. Chrome, Firefox, and Edge all work.
3. Enter the UI address in the address bar.
4. You land on the InDro UI home page (the Dashboard).

`[IMAGE: InDro UI home page]`

**Check the top status bar before operating the robot.**

| Indicator | Healthy | What a bad reading means |
| :---- | :---- | :---- |
| Latency, top left of the status row | Tens of milliseconds | See Known Issue 5. Do not drive on a reading in the thousands. |
| Emergency stop banner | Replaced by a green **START** button | Motion is disabled. Check both physical E-Stops, then press START. |
| Controller icon | Green | No gamepad detected. See 9.6. |
| Battery icon | Not near empty | Self-explanatory |

Pressing **START** can re-enable the arm as well as the base. Clear the area around the arm before pressing it.

### 9.2 Viewing ROS topics

1. Open the InDro UI.
2. Find the **Customize** button in the top-right corner. This is where all available ROS topic data is listed.
3. Click **Customize.** A list of available ROS topics appears. Select the ones you want to monitor.
4. Selected topics begin displaying live data.

If nothing appears, the topic is not publishing. Confirm from an SSH session before assuming the UI is at fault:

```
ros2 topic list
```

`[IMAGE: Customize menu with ROS topic list]`

### 9.3 LiDAR point cloud

1. Open the InDro UI.
2. Click **Customize** in the top-right corner.
3. In the search bar, type `LiDAR` to filter the options.
4. Select **Rslidar Points.**
5. The point cloud appears on the Dashboard.

`[IMAGE: LiDAR point cloud on the dashboard]`

### 9.4 Rear camera

1. Open the InDro UI.
2. Click the three-line menu in the top-left corner and select **Pilot View.**
3. Click **Play** to start streaming the rear camera feed.
4. Click the checkmark next to Play to pin the feed to the Dashboard for quick access.
5. Click the square button in the bottom-right of the camera window to maximise the view.

`[IMAGE: Pilot View with rear camera feed]`

### 9.5 ZED X camera

The ZED is not running by default. Start it from an SSH session on the robot, inside tmux so it survives a dropped connection:

```
tmux new -s zed
```

```
front
```

Then detach with `Ctrl+B`, release, `D`. `tmux attach -t zed` gets you back to it.

`front` launches the ZED driver from `~/colcon_ws`. The vendor documentation says to type it in the dashboard Console. It also works directly in SSH, which is what ERAU uses. Startup has succeeded when the log reports "Starting Positional Tracking" and an initial camera pose.

**Topic names.** The vendor documentation lists `image_raw_color` and `image_raw_grey`. Those topics do not exist. The actual topics are:

| Want | Topic |
| :---- | :---- |
| Colour image | `/zed/zed_node/rgb/image_rect_color` |
| Left or right eye | `/zed/zed_node/left/image_rect_color`, `/zed/zed_node/right/image_rect_color` |
| Both eyes side by side | `/zed/zed_node/stereo/image_rect_color` |
| Depth map | `/zed/zed_node/depth/depth_registered` |
| 3D point cloud | `/zed/zed_node/point_cloud/cloud_registered` |
| IMU | `/zed/zed_node/imu/data`, `/zed/zed_node/imu/data_raw` |
| ZED visual odometry | `/zed/zed_node/odom` |

Each image topic also has a `/compressed` version, which is the one to record.

To view a feed, open **Customize** on the dashboard and search `zed`. To enlarge it, use the widget's maximise control if it has one, or press `F11` for browser full screen.

**Measured rates:** images at about 21 to 23 Hz while recording, and the IMU at about 197 Hz.

The ZED needs visible texture to compute depth. A blank wall gives poor depth, which looks like a fault but is not.

If `front` fails with a `SyntaxError` in `zed_camera.launch.py`, see 11.3.

`[IMAGE: ZED camera feed on the dashboard]`

### 9.6 Driving with an Xbox controller

1. **Check controller status.** The controller icon in the top corner of the InDro UI is **red** when no controller is connected.
2. **Connect the controller.**
   * **Wired (recommended).** Plug the controller into your laptop by USB.
   * **Bluetooth (optional).** Works, but wired is recommended for teleoperation because it is more stable and avoids data loss.
3. **Activate it.** Move the joystick slightly in any direction. This signals to the robot that the controller is ready.
4. **Verify.** The icon changes from red to green. Green confirms the connection.
5. **Drive.** Hold **LB** and use the joystick to navigate.

`[IMAGE: InDro UI showing red vs green controller icon]`

**Confirmed behaviour, 9/14/26.** The Gamepad tab reports the detected device. A working detection looks like this:

```
ROBOT: Connected.
Gamepad: HID-compliant game controller (STANDARD GAMEPAD Vendor: 045e Product: 0b13)
```

`045e` is Microsoft and `0b13` is an Xbox Series controller. The tab then renders 4 axis bars and 17 button indicators, numbered from 0. Moving the sticks should animate the axis bars immediately. **That readout is the fastest way to localise a fault:**

* Bars do not move: the browser is not receiving input. Click once on the page so it has focus, then retry. Browsers suppress gamepad events until the page itself has seen input.
* Bars move but the robot does not: input is reaching the browser and the fault is downstream. Work through the checklist below.

**Confirmed driving, September and October 2026.** Driving with an Xbox controller through the Gamepad tab works, with LB as the deadman. The `xbox_teleop` container turns the controller input into drive commands on **`/mux/cmd_vel`**, which is the drive topic (`/cmd_vel` does not exist on this unit). `/joy` carries the raw stick and button input. Both can be recorded (9.10).

**AgileX remote.** Also confirmed working. It drives the chassis over its own radio, so it keeps working if the Jetson hangs or the Wi-Fi drops. Wheel odometry (`/agilex_odom`) still records RC driving: a 7 m move registered correctly. RC commands do not appear on any ROS topic.

**When the controller is detected but the robot does not move:**

1. **Is LB held?** LB is the deadman. Confirmed on this unit during Day 2 drive testing. The wheels will not turn without it held, and this is the most common cause.
2. **Was START pressed?** Motion stays disabled until the green START button in the UI is clicked.
3. **Is the AgileX transmitter still on, and where is SWB?** The chassis accepts CAN commands from the Jetson only in the correct transmitter mode. `[VERIFY: SWB position was never confirmed on this unit. The symptom that prompted the question, headlights responding to the remote while the sticks did not drive, was never reproduced after the chassis fault cleared.]`
4. **Confirm commands are leaving the computer.** From an SSH session, with the sticks moving:

```
ros2 topic echo /mux/cmd_vel
```

Values appearing means the software chain works and the problem is at the chassis. Nothing appearing means the UI is not publishing.

> `[TODO: MISSING SOURCE]` The vendor site linked to a separate Gamepad page for full button mappings. Not captured. LB as deadman is confirmed. The remaining mappings should be recorded here once read off the hardware. To do this, watch the numbered indicators in the Gamepad tab while pressing one button at a time.

### 9.7 Arm and gripper

> **Arm work is on hold.** The lab lead has paused it. The findings below are recorded so work can resume without repeating them.

#### 9.7.1 Normal operation

1. Open the InDro UI, click the three-line menu, and select **Missions Page.**
2. Click **Restart RViz** to load the arm model.
3. The arm model appears in the RViz window.
4. Use RViz to drag the arm to the target position. **This plans the motion, it does not execute it.**
5. Click **Plan and Execute.** The physical arm and the UI model both move to the planned position.
6. For the gripper, click **Control**, then **Open** or **Close.**

> Clear the area around the arm before clicking **Plan and Execute.** The planner will move the physical arm to whatever pose you set in RViz.

`[IMAGE: Missions Page with RViz arm model]`

#### 9.7.2 What RViz is and is not

RViz is a **visualiser**. It draws the robot's state and lets you set goal poses. It is not a control panel and it does not talk to the arm's controller directly.

This distinction is significant because the joint values shown in the MoveIt panel are **goal state, not measured state**. They show where the planner intends to go, not where the arm physically is. During Day 3 troubleshooting this was misread as live encoder feedback, which delayed troubleshooting. If you need actual joint positions, read them from the arm's own interface, not from the RViz panel.

The corollary: **you cannot correct a bad zero reference through RViz.** Moving sliders there does not write to the controller's encoder reference.

#### 9.7.3 Recovering a lost zero reference

If the arm has collapsed under power loss (Section 1.3), its controller no longer knows where the joints physically are. It will drive to a "home" that does not match reality.

The fix is a manual per-joint re-teach against the arm's own controller, using UFactory Studio. RViz and the InDro dashboard cannot do this.

1. Connect to the arm control box at `192.168.42.110`.
2. Connect UFactory Studio to that address.
3. Clear faults. In Studio, use **Enable Robot**. Via the SDK, the sequence is `clean_error()`, `clean_warn()`, `motion_enable(true)`, `set_state(0)`.
4. **Physically support the arm's weight.** The joints will free-drive.
5. Enter teach or free-drive mode and move each joint to its scribed physical zero mark.
6. Re-teach zero per joint in Studio.
7. Power cycle the arm at its XT60 connector and verify home.

> Powering the arm off and repositioning it by hand does **not** fix this. With power off the controller is not reading encoders, so it resumes from its last stored reference regardless of where the arm was physically left. The re-teach is the only path.

#### 9.7.4 What is known about the arm's control path

UFactory Studio still does not connect ("server is not ready"). Investigation from the Jetson found:

* `192.168.42.110` answers ping in under 1 ms. Its MAC address is `00:80:6f:4e:af:f5`, a prefix registered to ONELAN Ltd., a signage company. This likely indicates a third-party network chip and does not identify the arm's manufacturer.
* No TCP port tested on `.110` is open: 22, 80, 443, 502, 5000, 5001, 8000, 8080, 9000, 18333, and 30000 to 30003. So there is no SSH, web interface, Studio server, or xArm SDK port. `ssh unitree@192.168.42.110` is refused.
* There is no arm container, running or stopped, no arm ROS node, and no arm topic. RViz loads an arm model, but nothing in the ROS graph connects it to hardware.
* A packet capture on every interface while the arm was commanded from RViz recorded zero packets to or from `.110`.
* `/dev/ttyUSB0`, `/dev/ttyUSB1`, and `/dev/ttyACM0` exist, but no process had them open.
* The NUC has no Docker and no robot software.

These results conflict with the arm being observed to move when commanded from RViz. This is unresolved. When work resumes:

1. Unplug the arm's Ethernet cable and ping `.110`. If it still answers, `.110` is not the arm.
2. Read the model and serial number off the control box label (Known Issue 8).
3. Capture all traffic with no host filter while commanding a move, watching the arm: `sudo tcpdump -n -i any`

### 9.8 Alternative control path: rosboard and rosbridge (ERAU)

This path was validated at ERAU during commissioning and is independent of the InDro UI. It can be used when the InDro Controller container is not functioning, or for testing directly against the ROS graph.

Relevant nodes, running from `~/colcon_ws`:

| Node | Role |
| :---- | :---- |
| `/scout` | Chassis driver |
| `/rosbridge_websocket` | WebSocket bridge |
| `/rosboard_node` | Web dashboard backend |
| `/teleop_twist_joy_node` | Gamepad input handler |

**Procedure:**

1. Load the web controller interface on port **8888**.
2. Toggle the main connection switch.
3. Connect a gamepad via the browser Gamepad API. A Logitech Cordless RumblePad 2 has been verified working, and axis feedback is visible on the dashboard.
4. **Hold LB (L1).** This is the deadman switch. The wheels will not turn without it held.
5. Actuate the joystick to drive.

Expected behaviour at rest: zero velocity, 0 A current draw, no error codes.

**Port reference.** `rosboard` serves the dashboard on 8888. `rosbridge_websocket` listens separately on **9090** and is what any custom client should target. Confirm both are listening:

```
sudo ss -tlnp | grep -E "8888|9090"
```

### 9.9 Alternative control path: standalone gamepad page (ERAU)

Built at ERAU as a fallback for when the InDro UI's Gamepad tab will not bind a controller. The file is `scout-teleop.html`, kept alongside this manual.

It is a single HTML file that runs entirely in a browser. Nothing is installed on the client machine and nothing is changed on the robot. It reads the gamepad through the browser Gamepad API, translates the browser's axis layout into the Linux `joydev` layout that `joy_node` would produce, and publishes to `rosbridge_websocket` over a plain WebSocket.

**Why it publishes `/joy` and not `/cmd_vel` by default.** Publishing `/joy` feeds the robot's existing `/teleop_twist_joy_node`, so the configured deadman button and speed limits still apply. Publishing `/cmd_vel` bypasses them. The page supports both, but `/cmd_vel` mode carries an explicit warning and enforces its own hold-to-drive button.

**Procedure:**

1. Confirm rosbridge is listening on 9090 (see 9.8).
2. Open the file in a browser.
3. Plug in the controller and press a button on it.
4. Confirm the on-screen sticks track your input **before** connecting.
5. Enter the robot address and press **Connect.** The page sends neutral input at this stage and the robot will not move.
6. Press **Arm.**

Safety behaviour built in: the page sends neutral input continuously when not armed, and disarms automatically on window blur, tab hide, gamepad disconnect, or socket close. Space or Escape stops it immediately.

The live axis readout can also be used to record the button mappings still missing under Known Issue 5.

**Status, 10/23:** this page has never been used on the robot, since the dashboard's Gamepad tab works. If it is used in direct mode, set the topic to `/mux/cmd_vel`.

### 9.10 Recording rosbags

A rosbag records ROS topics to disk for analysis later, with the robot off. Bags go in `~/bags` on the Jetson. Disk space is not a constraint: the Jetson had 825 GB free of 937 GB on 9/22.

**Before recording:**

1. The clock is synchronised (`timedatectl`).
2. The ZED is running (9.5).
3. You are inside tmux, so a dropped connection does not stop the recording.

**Steps, on the robot:**

```
tmux new -s bag
```

If it says the session already exists, use `tmux attach -t bag` instead.

```
cd ~/bags
```

Set the standard topic list:

```
TOPICS="/tf /tf_static /rslidar_points /zed/zed_node/imu/data /zed/zed_node/imu/data_raw /zed/zed_node/left/image_rect_color/compressed /zed/zed_node/right/image_rect_color/compressed /zed/zed_node/left/camera_info /zed/zed_node/right/camera_info /zed/zed_node/depth/depth_registered /zed/zed_node/odom /agilex_odom"
```

For a run driven with the Xbox controller, add the command topics:

```
TOPICS="$TOPICS /joy /mux/cmd_vel"
```

Check the list before recording. An outdated list caused `hallway_02_cmd` to be recorded without depth:

```
echo $TOPICS
```

Record, replacing `NAME`:

```
ros2 bag record -o NAME $TOPICS
```

**Wait for `All requested topics are subscribed` before moving the robot.** If it does not appear within a few seconds, a topic is missing, usually the ZED. Stop with `Ctrl+C` and fix it. Otherwise the bag records without that sensor and gives no other warning, which is what happened to `hallway_01`.

Detach with `Ctrl+B`, then `D`. Drive the run. Then reattach and stop:

```
tmux attach -t bag
```

Press `Ctrl+C`, then check the bag:

```
ros2 bag info NAME
```

**A valid bag has:** every requested topic listed; the ZED's image, camera_info, and odom topics with equal counts; the LiDAR near 10 Hz; the IMU near 200 Hz; and at least one `/tf_static` message. Divide each count by the duration to get the rate.

The repository's `scripts/record_bag.sh` does the same and refuses to start if the clock is not synced or the ZED is not running. It had not been tested on the robot as of 10/23.

**Measured bandwidth:**

| Topic | Rate | Data rate |
| :---- | :---- | :---- |
| `/rslidar_points` | 10 Hz | 9.3 MB/s |
| `/zed/zed_node/depth/depth_registered` | about 28 Hz live | 65.5 MB/s |
| `/zed/zed_node/rgb/image_rect_color/compressed` | about 29 Hz live | 5.3 MB/s |

With depth, a bag grows by roughly 5 GB per minute. Without it, about 1 GB per minute. A 30-second test with depth recorded 1.9 GiB with no dropped messages.

**For driving runs:**

* Mark the start position and heading with tape, and start every run there.
* Hold still for 30 seconds at the start and end. The IMU needs a stationary window to settle.
* Drive at walking pace or slower, smoothly, without spinning in place.
* Finish back on the start mark, so the data can test loop closure.
* Record several short runs rather than one long one, and check each with `ros2 bag info` before the next.

**Naming.** Add `_cmd` to the name of any bag that contains `/mux/cmd_vel`. Never play those on the powered robot (1.4).

**Bags recorded so far:**

| Bag | Date | Length | Size | Contents |
| :---- | :---- | :---- | :---- | :---- |
| `test_30s` | 9/22 | 30 s | 1.9 GiB | Stationary validation, all sensors including depth |
| `hallway_01` | 9/29 | 168 s | 1.4 GiB | Hallway P route. LiDAR, wheel odometry, and transforms only; the ZED was down. |
| `straight_5m` | early Oct. | | | Tape-measured 5 m straight line |
| `hallway_02_cmd` | 10/14 | 532 s | 7.8 GiB | Hallway P route with the Xbox controller. LiDAR, both ZED eyes, IMU, ZED and wheel odometry, `/joy`, `/mux/cmd_vel`. No depth. |

### 9.11 Copying bags off the robot

Use Ethernet. Bags are large and the robot's Wi-Fi is slow. In PowerShell on your laptop, replacing `NAME`:

```
scp -r indro@192.168.42.42:~/bags/NAME "$env:OneDrive\ScoutBags"
```

A bag is a folder holding a `.db3` file and a `metadata.yaml`. Copy the whole folder; one without the other is useless.

Check the copy is identical. On the robot's copy:

```
ssh indro@192.168.42.42 "sha256sum ~/bags/NAME/NAME_0.db3"
```

On your copy:

```
Get-FileHash "$env:OneDrive\ScoutBags\NAME\NAME_0.db3"
```

The two strings must match, ignoring upper and lower case. Then wait for OneDrive to show a green check before closing the laptop.

If `$env:OneDrive` is empty, your school account may be `$env:OneDriveCommercial`. Check with `Get-ChildItem Env:OneDrive*`.

The lab's bags are on OneDrive, linked from the repository README. Bags are too large for Git.

### 9.12 Shutting down

1. Put the arm in a low, stable pose (1.3).
2. On the robot:

```
sudo shutdown now
```

3. Wait for the Jetson's lights to go dark.
4. Turn the ignition key off.

Cutting power with the Jetson running risks corrupting its disk. It also obscures real crashes: in `journalctl --list-boots`, a power cut and a crash appear identical (11.1).

---

## 10. Docker & development

All InDro Commander components run in separate containers.

### 10.1 Inspecting containers

List running containers:

```
docker ps
```

View a container's logs:

```
docker logs <container_name>
```

List available images:

```
docker images
```

`[IMAGE: docker images output]`

### 10.2 Working inside containers

Open a new shell in a running container:

```
docker exec -it <container_name> /bin/bash
```

Attach a terminal to a running container:

```
docker attach <container_name>
```

> Prefer `exec` over `attach`. `attach` connects your terminal to the container's main process, so pressing `ctrl+c` will kill the process running inside it. `exec` opens a separate shell and is safe to exit.

Create a new container from an existing image:

```
docker run -it --net=host --name <name_of_container> --privileged <REPOSITORY>:<TAG> /bin/bash
```

This creates the container and drops you into a terminal inside it.

### 10.3 Source layout

All Docker source lives in `~/docker` on the Jetson:

```
~/docker
  docker-compose.yaml
  indro-controller/
    cyclonedds.xml
    docker-compose.yaml
    Dockerfile
    README.md
    ros_entrypoint.sh
  oak_ros2/
    (same structure)
  robosense_ros2_docker/
    (same structure)
  rtsp-webrtc/
    config.json
    docker-compose.yaml
    README.md
  swiftnav_ros2_docker/
    (same structure)
```

Each directory carries its own `cyclonedds.xml`. These are the reference configs for reconstructing Section 7.2.

`[IMAGE: tree output of ~/docker]`

### 10.4 Development practice

Build a new container rather than modifying the ones InDro provided. Start from an already-built image (`docker images`) or from scratch following the Docker tutorial. Editing vendor containers in place makes it impossible to distinguish vendor code from local changes and prevents a clean rollback.

---

## 11. Known issues & open questions

### 11.1 Jetson unresponsive while powered

**Severity: high. Observed three times in a single session, 9/14/26.** It is the highest-priority open issue on the unit.

**Symptom.** `192.168.42.42` and `10.13.46.42` both stop answering. Pings fail with `Destination host unreachable`, and the reply comes from the **pinging machine's own address**, not from a remote host. The Jetson's status LED strobes orange. Its port on the GS105 shows a normal link light.

**What that combination means.** An active switch link with no ARP reply indicates that the link layer is working and the host is not responding. Since the local LAN address fails as well as the ZeroTier one, both the VPN and the lab internet are ruled out. This is the host itself, hung or crashed, and network troubleshooting will not resolve it.

**Diagnostic sequence.** Run from the NUC or any machine on the robot's subnet.

Check the LAN address first:

```
ping 192.168.42.42
```

If that fails, check the other onboard machine to confirm the subnet itself is alive:

```
ping 192.168.42.43
```

NUC answers and Jetson does not means the Jetson is down, not the network.

**Recovery.** A brief press of the power button sends a shutdown signal to software that is already hung, and does nothing. A full power cycle is required.

1. Hold the Jetson's power button for a **full 10 seconds.** Wait for lights and fans to go completely dead.
2. Wait 15 seconds.
3. Press once to start.
4. Wait 3 minutes for boot.
5. Ping again.

If the button has no effect, pull the DC barrel connector for 15 seconds. Most of these boards auto-boot when power returns.

**Root cause: unknown.** The next person to bring it back up should capture the logs before doing anything else:

```
journalctl --list-boots
```

Then read the tail of the previous boot:

```
journalctl -b -1 -e
```

A log that ends abruptly with no shutdown messages indicates a hard hang or power loss. A kernel panic, an OOM kill, or a thermal event will be named explicitly. Candidate causes are thermal (the bay is enclosed and the Jetson sits under another module), power delivery from the 24 V rail, and storage. Until that log is read, the procedures above address the symptom only.

**Update, 10/23.** Crash logs have still not been captured. Two practices are required to diagnose the next occurrence: always shut down with `sudo shutdown now` (9.12), so that a crash is distinguishable in `journalctl --list-boots`; and read `journalctl -b -1 -e` immediately after recovering, before anything else.

### 11.2 Remaining items

| # | Item | Detail | Status and action |
| :---- | :---- | :---- | :---- |
| 1 | ZeroTier network ID conflict | The vendor page gave two IDs | **Resolved.** `3850b435d4b223c6` works (6.2). |
| 2 | Jetson model naming | Text says Orin AGX, wiring diagram says Orin NX | **Resolved.** Orin NX, confirmed 10/23 with `cat /proc/device-tree/model`. |
| 3 | `Scout_mini` container | Listed by the vendor | **Resolved.** It does not exist; the chassis driver is `base_platform` (3.3). |
| 4 | CycloneDDS config missing | The XML and environment variables were never captured | Open. Reconstruct from the `cyclonedds.xml` files under `~/docker/`. |
| 5 | Gamepad mappings incomplete | LB as deadman is confirmed for both the Logitech and Xbox controllers | Open for the rest. Read them off the Gamepad tab's numbered indicators. |
| 6 | `error_code: 128` | Seen on Day 1, cleared by a power cycle | Not seen since. |
| 7 | Router page protocol | Vendor says `http://`, the log recorded `https://` | Both work. Failures are caused by Firefox's automatic HTTPS upgrade. |
| 8 | Arm manufacturer conflict | Branded InDro, software UFactory, vendor login `unitree` | Open, on hold. Read the model and serial off the control box label. |
| 9 | UFactory Studio cannot connect | "Server is not ready" | Open, on hold. See 9.7.4. |
| 10 | Dashboard latency implausible | Reads 16000 to 17000 ms against a 1.3 ms ping, drifting upward | Open. Likely tied to the Jetson's clock at boot (14). Re-check once the clock is synced. |
| 11 | Pin-hole camera address | Vendor says `.201`; a scan finds a camera at `.101` and nothing at `.201` | Open. Confirm on the hardware. |
| 12 | No arm node in the ROS graph | No arm container, node, or topic | Confirmed (9.7.4). On hold. |
| 13 | `rtsp-webrtc` restart loop | Restarting continuously whenever observed | Open. Check `docker logs rtsp-webrtc --tail 30`. Affects dashboard camera streaming, not ROS topics. |
| 14 | Jetson clock starts at 1970 | Every SSH login after a cold boot shows "Last login: Thu Jan 1 00:00 1970" | Open. Likely a depleted real-time-clock battery. Until it is replaced, check `timedatectl` before every recording. |
| 15 | GNSS has no driver | u-blox receiver and antenna installed; the vendor's Swift Navigation container does not exist | Open. The lab lead says the lab has other GPS units that can mount on Scout; coordinate with Dheer. |
| 16 | RViz only works over ZeroTier | The panel connects to `10.13.46.42` directly | Known limitation (8.1). Use ZeroTier for RViz. |
| 17 | `/scout_status` cannot be recorded from the host | Its message type exists only in the `base_platform` container | Known limitation (3.1). Record from inside that container if it is ever needed. |
| 18 | `indro-controller` restarted on its own | Seen once: up 4 minutes while every other container was up 35 | Open. Watch for repeats. |
| 19 | Xbox control drops outside the lab | Dashboard control lost once the robot left lab Wi-Fi range | Workaround in 5.5. Cause not confirmed. |

### 11.3 Minor issues and fixes

Each of these presents as a more serious fault than it is.

* **A broken retaining clip on an Ethernet plug** gives `Media disconnected` with no other symptom, on a port that is otherwise functional. Check the clip before suspecting the port, the cable run, or the switch. Discard the cable.
* **Firefox auto-upgrades to HTTPS** and then fails with "Unable to connect" on every InDro service, all of which are plain HTTP. If a page will not load, check the address bar for an `s` that you did not type. Forcing the port, `http://192.168.42.42:80/console`, defeats the upgrade.
* **The InDro UI's Gamepad tab renders empty** until the browser has seen gamepad input. This looks like a broken feature and is not. Click on the page, then press a button on the controller.
* **Charge-only USB cables** are common with Xbox controllers and produce a charging light with no data at all. If a wireless controller is in use, Windows must report **Connected**, not merely Paired.

* **The ZED will not start, and `front` fails** with `SyntaxError: '(' was never closed (zed_camera.launch.py, line 303)`. On 9/29 this turned out to be an accidental edit made on 9/22 that deleted a closing parenthesis. The ZED kept running until its next relaunch, so the break surfaced a week after the edit. To fix it, on the robot:

  ```
  cd ~/colcon_ws/src/zed-ros2-wrapper
  ```

  ```
  git diff zed_wrapper/launch/zed_camera.launch.py
  ```

  If the diff shows only the accidental change, restore the file:

  ```
  git restore zed_wrapper/launch/zed_camera.launch.py
  ```

  Coordinate with the team before editing files on the robot.
* **The dashboard shows a white screen** after choosing an option in Customize. The broken layout is saved in the browser, so refreshing does not help. Open the dashboard in a private window to confirm, then clear the site's data: `F12`, then **Application > Storage > Clear site data**.
* **A bag records without a sensor**, with no error. The recorder skips topics that are not publishing. Wait for `All requested topics are subscribed` before driving (9.10).
* **`ros2 bag info` says the bag does not exist.** You are in your home folder; use the full path, `~/bags/NAME`.
* **A command is not found.** Check the prompt. `PS C:\...` is your laptop; `indro@indro-commander:~$` is the robot. `ros2` only exists on the robot.
* **tmux says "duplicate session".** That session is already running. Reattach with `tmux attach -t NAME`.

---

## 12. ERAU commissioning log

A record of what was verified on this unit, as distinct from what the vendor documentation claims.

### Day 1: System inspection and discovery

* **Connectivity.** Established SSH access via VS Code over Ethernet. Robot at `192.168.42.42`, router at `192.168.42.1`.
* **Environment audit.** Confirmed Ubuntu 22.04 LTS, ROS 2 Humble, Docker containerisation, and CAN communication bus. (This was the container environment. See Section 3.1.)
* **Topic mapping.** Identified active ROS topics covering movement commands, Xbox joystick controls, lighting, battery status, and wheel odometry.
* **Initial diagnostics.** Observed `error_code: 128`. Performed physical E-Stop inspection. Read-only session, no files modified.

### Day 2: CAN diagnostics, fault resolution and drive testing

* **CAN dump verification and fault clear.** Evaluated low-level CAN bus telemetry against ROS parameters. Confirmed `vehicle_state: 0` and `error_code: 0` following a full system power cycle.
* **Driver state baseline.** Confirmed that `driver_state: 64` across all four motor drivers is the expected normal idle baseline, not a drive-disabled fault flag. It resembles a fault code but is not one.
* **Active node verification.** Verified `/scout`, `/rosbridge_websocket`, `/rosboard_node`, and `/teleop_twist_joy_node` running from `~/colcon_ws`.
* **Web controller and gamepad binding.** Loaded the INDRO Web Controller on port 8888, toggled the main connection switch, connected a Logitech Cordless RumblePad 2 via the browser Gamepad API, and verified axis feedback on the dashboard.
* **Physical drive test (successful).**
  * Confirmed the deadman switch requirement: holding LB (L1) while actuating the joystick commands active wheel rotation.
  * Validated smooth low-speed forward and reverse motion.
  * Confirmed full return to zero velocity and 0 A current draw at rest, with no error codes.

### Day 3: Arm fault, network forensics, and host failure

* **Arm collapse.** The robot was powered down with the arm raised. The joints have no brakes, so the arm fell under gravity and the controller lost its zero reference. Root cause understood and the recovery path identified (Section 9.7.3), but the repair was blocked at its first step, connecting to the arm's controller.
* **Incorrect approach.** The correction was first attempted through the InDro dashboard's RViz and MoveIt panels. These are a planning and visualisation layer only. The joint values displayed are goal state, not measured state, and setting them does not write to the arm's encoder reference. See Section 9.7.2.
* **Subnet mapping.** Built a complete host inventory of `192.168.42.0/24` via `nmap -sn` after the router's DHCP lease page showed no active IPv4 leases. Six hosts found. Five identified at the time; `.110` was left unresolved and has since been confirmed as the arm.
* **Misidentification corrected.** `192.168.42.43` was initially assumed to be the arm and Studio was repeatedly pointed at it. It is the NUC. Confirmed via `nm-connection-editor`, which also revealed the NUC holds a static address rather than a DHCP lease.
* **UFactory Studio.** Downloaded and launched `UfactoryStudio-Linux-1.0.1.AppImage` on the NUC. Server discovery found nothing and manual connection attempts to `.42`, `.43`, and `.110` all returned "server is not ready". Arm control box was power cycled at the XT60 with no change; its LEDs read normal throughout. Unresolved, see Known Issue 9.
* **Jetson unreachable.** First occurrence of the failure now documented as Known Issue 11.1. Established that ZeroTier and the lab internet were not implicated, since the local LAN address failed identically.

### Day 4: Client setup, gamepad binding, and recurring host failure

* **Jetson recovered.** A full power cycle restored the host. Both `192.168.42.42` (1.3 ms) and `10.13.46.42` (0 percent loss, first packet 26 ms during peer handshake, then about 2 ms) confirmed healthy afterwards.
* **Client access established.** Windows laptop connected over Ethernet to the back-panel port. InDro Controller UI reachable and usable.
* **Network topology clarified.** Confirmed that joining a nearby Wi-Fi network whose name resembles the robot's does not place a client on `192.168.42.x`, since the router operates in client mode against the lab hotspot. Ethernet and ZeroTier are the only two supported client paths. A ceiling-mounted lab access point was briefly patched into the robot's LAN and then removed; see the warning in Section 5.1.
* **ZeroTier client joined.** Laptop joined `3850b435d4b223c6` and reached `ACCESS_DENIED`, pending manual authorisation. Request sent to InDro support.
* **Gamepad detected.** Xbox controller (vendor `045e`, product `0b13`) successfully bound through the InDro UI Gamepad tab, reporting as a standard gamepad with 4 axes and 17 buttons. Both status indicators green. Drive test not completed this session.
* **Dashboard latency anomaly.** UI reported roughly 16000 to 17000 ms against a measured 1.3 ms ping. Clock skew investigated and ruled out. Logged as Known Issue 10.
* **Jetson unreachable, twice more.** Two further occurrences of the Known Issue 11.1 failure, the second while the operator was away from the bench. Three occurrences in one session establish this as a recurring fault and the highest-priority open item. Crash logs were not captured.

### Day 5: Remote access and software inventory (September)

* **ZeroTier working.** A laptop was authorised by InDro support and reached the Jetson at `10.13.46.42`. RViz was found to work only over ZeroTier.
* **Robot Wi-Fi confirmed** as a way to connect, contradicting the previous revision.
* **Software inventory.** Eleven containers running, two stopped, with only one name matching the vendor list (3.3). The host was found to install Ubuntu 22.04 packages, not 20.04 as documented. The NUC has no Docker.
* **GNSS found.** A u-blox receiver on USB with its antenna mounted, but no driver.
* **Arm investigation.** No open ports on `.110`, no arm software in the ROS graph, and no traffic during a commanded move (9.7.4). The lab lead then paused arm work.
* **Dashboard white-screen bug** found and worked around (11.3).

### Day 6 (9/22): Recording baseline

* Clock confirmed synced before recording. The "Last login 1970" message pointed to the real-time-clock issue (Known Issue 14).
* Disk space measured: 825 GB free of 937 GB.
* Bandwidth measured for the LiDAR, ZED depth, and compressed colour (9.10).
* 30-second test bag recorded and validated: all topics present, no dropped messages. `/scout_status` found unrecordable from the host.

### Day 7 (9/29): ZED recovery and first hallway run

* The ZED wouldn't start. Traced to an accidental edit of its launch file and restored with Git (11.3).
* tmux adopted, so the camera and recordings survive a dropped connection.
* Wheel odometry confirmed to record AgileX remote driving.
* `hallway_01` recorded along the P route. The ZED had stopped before recording, so the bag holds only LiDAR, wheel odometry, and transforms.

### Day 8 (early October): Straight-line reference

* `straight_5m` recorded along a tape-measured 5 m line, as a ground truth for wheel odometry error.

### Day 9 (10/14): Full hallway run with commands

* Xbox control used on the robot's own Wi-Fi.
* `hallway_02_cmd` recorded: 532 s, 7.8 GiB, LiDAR at 10.0 Hz, ZED at 21.4 Hz in lockstep, IMU at 197 Hz, wheel odometry at 50 Hz, and the commanded velocity. Depth was missing because an older topic list was set.

---

## 13. External references

**Vendor documentation**

* Scout 2.0 base platform (AgileX)
* InDro Manuvr Arm
* UFactory xArm documentation, relevant if Known Issue 8 resolves toward UFactory

**GitHub**

* `scout_ros`, Scout ROS 1 driver
* `scout_ros2`, Scout ROS 2 driver
* `ugv_sdk`, AgileX UGV SDK (low-level CAN protocol)

**Other**

* Docker overview: `https://docs.docker.com/get-started/overview/`
* InDro support: `support@indrorobotics.com`
* UFactory support: `support@ufactory.cc`

> The vendor documentation links above pointed to pages on the abandoned site. Replace with archived copies or the AgileX and InDro equivalents when located.

---

## Appendix: Figure index

Placeholders in the order they appear. Suggested filenames assume an `images/` directory alongside this document.

| Section | Figure | Suggested filename |
| :---- | :---- | :---- |
| 1.2 | E-Stop button location | `estop-location.png` |
| 2.1 | Full robot, exterior | `robot-exterior.png` |
| 2.2 | Commander box interior, labelled | `commander-interior.png` |
| 2.2 | Exterior labelled, showing arm, gripper, LiDAR, ZED X | `robot-labelled.png` |
| 2.4 | Wiring diagram | `wiring-diagram.png` |
| 3.3 | `docker ps` output | `docker-ps.png` |
| 4.2 | Rear panel, power button and voltage display | `power-button.png` |
| 5.1 | Router login page | `router-login.png` |
| 5.2 | Hotspot Manager, SRGE_Local connected | `hotspot-manager.png` |
| 6.3 | ZeroTier system tray icon (Windows) | `zerotier-tray.png` |
| 6.3 | Join ZeroTier Network dialog | `zerotier-join.png` |
| 6.4 | `zerotier-cli listnetworks` output | `zerotier-listnetworks.png` |
| 7.1 | `ifconfig` showing zt interface | `ifconfig-zt.png` |
| 9.1 | InDro UI home page | `indro-ui-home.png` |
| 9.2 | Customize menu, ROS topic list | `indro-ui-customize.png` |
| 9.3 | LiDAR point cloud on dashboard | `lidar-pointcloud.png` |
| 9.4 | Pilot View, rear camera feed | `pilot-view.png` |
| 9.5 | ZED camera feed on dashboard | `zed-feed.png` |
| 9.6 | Controller icon, red vs green | `controller-status.png` |
| 9.6 | Gamepad tab showing detected device and axis bars | `gamepad-detected.png` |
| 9.7 | Missions Page, RViz arm model | `missions-rviz.png` |
| 10.1 | `docker images` output | `docker-images.png` |
| 10.3 | `tree` output of `~/docker` | `docker-tree.png` |
| 11.1 | Jetson status LED, strobing orange fault state | `jetson-fault-led.png` |
| 11.1 | Failed ping showing `Destination host unreachable` | `ping-host-unreachable.png` |

To insert an image, replace the placeholder line with:

```
![E-Stop button location](images/estop-location.png)
```

**End of document.**
