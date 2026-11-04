# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

BentoBot is a two-wheel differential-drive robot used to test custom ROS packages for mapping, localization and path planning. The repo root is a **colcon workspace** for ROS 2 Foxy (Python 3 `rclpy`, C++14 + Eigen3). It runs inside a Docker container on the robot's Jetson Nano (developed over VSCode Remote-SSH). A Teensy 4.0 microcontroller handles motors, encoders and IMU; its firmware lives in `src/firmware/` as PlatformIO projects (`COLCON_IGNORE`d).

## Build & Run

Everything ROS runs in Docker (`Dockerfile`, `docker-compose.yml`, `docker/entrypoint.sh`). The image is `ros:foxy-ros-base` (arm64), and the `colcon build` happens at image build time.

```bash
docker compose build
docker compose up                 # full robot bringup: teleop.launch.py
docker compose run --rm dev       # shell with src/ bind-mounted; build/ install/ log/ land in the repo root
# inside the dev container
colcon build && source install/setup.bash
ros2 launch bentobot_bringup teleop.launch.py
ros2 launch bentobot_description bentobot_description.launch.py
ros2 launch bentobot_description gazebo_world.launch.py   # needs ros-foxy-gazebo-ros-pkgs, which is not in the image
```

- `teleop.launch.py` starts `rosbridge_server` (port 9090, Foxglove "Rosbridge" connection), the `base_link → base_laser` static TF, `joy_linux`, teleop, MCU bridge, the LD06 lidar, odom EKF and `slam_toolbox` (async).
- Foxy is EOL. The base image's apt source is the frozen `snapshots.ros.org/foxy/final`. Some packages were never released for Foxy (`foxglove_bridge`, hence rosbridge). `joy_linux` is used instead of `joy` because Foxy's `joy` 3.x is SDL2-based and can number axes differently.
- The LD06 driver (`ldlidar_stl_ros2`, LDRobot) is cloned at a pinned commit (`LDLIDAR_COMMIT` in the Dockerfile) into a separate underlay workspace, `/opt/ldlidar_ws`, so bind-mounting `src/` doesn't hide it.
- To add an apt dependency, add it to the Dockerfile's apt list and to the package's `package.xml`. rosdep isn't used.
- There are no automated tests or linters.

Firmware (each subfolder of `src/firmware/` is an independent PlatformIO project targeting `teensy40`; run on the host, not in Docker):

```bash
cd src/firmware/full_sensor_suite
pio run              # build
pio run -t upload    # flash Teensy (teensy-cli)
pio device monitor -b 115200
```

## Architecture / Data Flow

```
joy_linux --/joy--> bentobot_teleop --/mcu_cmd (Twist)--> bentobot_mcu_bridge <--serial 115200--> Teensy (full_sensor_suite)
                                                                 |
                                             /mcu_info (MCUInfo) v
                                                         bentobot_odom (EKF) --/odom + TF odom->base_link--> slam_toolbox (/scan from LD06)
```

- **Serial protocol (bridge ↔ firmware)** — plain text lines. Host → MCU: `"<lin_vel> <ang_vel>\n"`. MCU → host: `"<left_vel> <right_vel> <imu_gyro_z>\n"` at ~100 Hz. Changing either side requires changing `bentobot_mcu_bridge/mcu_bridge_node.py` and `full_sensor_suite.cpp` together. The bridge's `serial_port` parameter defaults to the Teensy's `/dev/serial/by-id/usb-Teensyduino_...` path. A background thread does the blocking serial reads, and the executor handles `mcu_cmd`.
- **Firmware (`full_sensor_suite`)** — quadrature encoder ISRs → per-wheel `MotorKalmanFilter` (velocity estimate) → per-wheel `MotorPID` (PID + exponential feed-forward `FF_A`, `FF_B`). It converts (v, w) to wheel velocities itself. The feed-forward constants come from `single_motor_feed_forward_calculator.py` plus the `*_test.csv` calibration data in `src/firmware/`. The other firmware folders are single-purpose bring-up/test sketches. `lib/MotorKalmanFilter` and `lib/MotorPID` are **copied** (not shared) between `full_sensor_suite` and `single_motor_calibration`, so keep them in sync.
- **Odometry (`bentobot_odom`)** — `Bentobot::KalmanFilter` is an EKF with a 5-state `[x, y, θ, v, w]`, a Runge-Kutta motion model, and two measurement updates: wheel odom (v, w) and IMU (w). It is ROS-agnostic, pure Eigen. Noise params are declared on the `bentobot_odom` node and loaded from `bentobot_bringup/config/bentobot_config.yaml` (the code has its own defaults). It publishes `/odom` and the dynamic `odom → base_link` TF.
- **Mapping** — `slam_toolbox` replaces gmapping. Its config is `bentobot_bringup/config/slam_toolbox_config.yaml` and uses TF only, not `/odom`.
- **Shared physical constants** are duplicated across layers: `WHEEL_BASE_LENGTH = 0.16` (firmware and `odom_node.cpp`), wheel diameter 0.068 and 680 ticks/rev (firmware). The `base_link → base_laser` static TF is in `bentobot_bringup/launch/description.launch.py`.
- `bentobot_interfaces` defines the `MCUInfo` message (`left_vel`, `right_vel`, `imu_ang_vel` + header). `bentobot_mcu_bridge` and `bentobot_odom` depend on it.
- The Gazebo sim (`bentobot_description/urdf/bentobot.gazebo`) uses the ROS 2 `gazebo_ros` diff-drive and ray-sensor plugins on `/cmd_vel`, `/odom`, `/scan`. Note that it uses `robot_footprint`/`lidar` frames, not the real robot's `base_link`/`base_laser`.
