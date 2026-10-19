# SCOUT2 Autonomy

ROS 2 autonomy and sensor integration for the SRGE Lab SCOUT2 rover.

## Project Goals

This repository contains development and documentation for:

- SCOUT2 rover control
- ZED X camera integration
- RoboSense LiDAR integration
- ROS 2 sensor data collection
- SLAM and mapping
- Nav2 autonomous navigation
- Future SPARTA sensor integration

## Hardware

- InDro Robotics SCOUT2
- NVIDIA Jetson Orin NX
- Stereolabs ZED X
- RoboSense LiDAR

## Status

Development in progress.

## ZED X Vision Demonstration

Successfully tested October 19, 2026.

### Start the camera (Commander Terminal 1)

    source /opt/ros/humble/setup.bash
    source ~/colcon_ws/install/setup.bash
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx

### Start the viewer (Commander Terminal 2)

    cd ~/github/scout2-autonomy
    python3 scripts/zed_live_view.py

### Connect from Windows PowerShell

    ssh -N -L 18081:127.0.0.1:8081 indro@192.168.42.42

Open http://localhost:18081 in your browser.

### Demonstrated Results

- Live RGB and depth images at approximately 15-16 FPS.
- Browser display with center crosshair and depth in meters.
- Verified changing distances by moving a backpack.
- Rover remained stationary; autonomous obstacle avoidance is not implemented.
