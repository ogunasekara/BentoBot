# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

BentoBot is a two-wheel differential-drive robot used to test custom ROS packages for mapping, localization and path planning. The repo root is a **catkin workspace** (ROS Melodic, Python 2 `rospy`, C++11 + Eigen3). It runs on the robot itself (developed over VSCode Remote-SSH). A Teensy 4.0 microcontroller handles motors, encoders and IMU; its firmware lives in `src/firmware/` as PlatformIO projects.

## Build & Run

```bash
# from repo root (catkin workspace)
catkin_make
source devel/setup.bash

# full robot bringup: foxglove bridge, URDF/TF, joystick teleop, MCU bridge, LD06 lidar, odom EKF, gmapping
roslaunch bentobot_bringup teleop.launch

# just URDF + robot_state_publisher / Gazebo sim
roslaunch bentobot_description bentobot_description.launch
roslaunch bentobot_description gazebo_world.launch
```

- `src/ldlidar_stl_ros` is a gitlink (submodule commit 964afad) with **no `.gitmodules` entry**, so it does not clone automatically; it must be cloned manually (LDRobot's `ldlidar_stl_ros` driver) for `lidar.launch` / `catkin_make` to work.
- External ROS deps used by launch files: `foxglove_bridge`, `joy`, `gmapping`, `tf2_ros`, `xacro`, `robot_state_publisher`, `joint_state_publisher`, `gazebo_ros`.
- There are no automated tests or linters.

Firmware (each subfolder of `src/firmware/` is an independent PlatformIO project targeting `teensy40`):

```bash
cd src/firmware/full_sensor_suite
pio run              # build
pio run -t upload    # flash Teensy (teensy-cli)
pio device monitor -b 115200
```

## Architecture / Data Flow

```
joy_node --/joy--> bentobot_teleop --/mcu_cmd (Twist)--> bentobot_mcu_bridge <--serial 115200--> Teensy (full_sensor_suite)
                                                                 |
                                             /mcu_info (MCUInfo) v
                                                         bentobot_odom (EKF) --/odom + TF odom->base_link--> gmapping (/scan from LD06)
```

- **Serial protocol (bridge ↔ firmware)** — plain text lines. Host → MCU: `"<lin_vel> <ang_vel>\n"`. MCU → host: `"<left_vel> <right_vel> <imu_gyro_z>\n"` at ~100 Hz. Changing either side requires changing `bentobot_mcu_bridge_node.py` and `full_sensor_suite.cpp` together. The bridge opens the Teensy at a hard-coded `/dev/serial/by-id/usb-Teensyduino_...` path.
- **Firmware (`full_sensor_suite`)** — quadrature encoder ISRs → per-wheel `MotorKalmanFilter` (velocity estimate) → per-wheel `MotorPID` (PID + exponential feed-forward `FF_A`, `FF_B`). It converts (v, w) to wheel velocities itself. The feed-forward constants come from `single_motor_feed_forward_calculator.py` plus the `*_test.csv` calibration data in `src/firmware/`. The other firmware folders are single-purpose bring-up/test sketches. `lib/MotorKalmanFilter` and `lib/MotorPID` are **copied** (not shared) between `full_sensor_suite` and `single_motor_calibration`, so keep them in sync.
- **Odometry (`bentobot_odom`)** — `Bentobot::KalmanFilter` is an EKF with a 5-state `[x, y, θ, v, w]`, a Runge-Kutta motion model, and two measurement updates: wheel odom (v, w) and IMU (w). Noise params are read from the `odom/*` rosparams loaded from `bentobot_bringup/config/bentobot_config.yaml` (the code has fallback defaults). It publishes `/odom` and the dynamic `odom → base_link` TF.
- **Shared physical constants** are duplicated across layers: `WHEEL_BASE_LENGTH = 0.16` (firmware and `odom_node.cpp`), wheel diameter 0.068 and 680 ticks/rev (firmware). The `base_link → base_laser` static TF is in `bentobot_bringup/launch/description.launch`.
- `bentobot_mcu_bridge` also defines the `MCUInfo` message (`left_vel`, `right_vel`, `imu_ang_vel` + header). `bentobot_odom` depends on it.
