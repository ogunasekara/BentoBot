# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

BentoBot is a two-wheel differential-drive robot used to test custom ROS packages for mapping, localization and path planning. The repo root is a **colcon workspace** for ROS 2 Foxy (Python 3 `rclpy`, C++14 + Eigen3). It runs inside a Docker container on the robot's Jetson Nano (developed over VSCode Remote-SSH). A Teensy 4.0 microcontroller handles motors, encoders and IMU; its firmware lives in `src/firmware/` as PlatformIO projects (`COLCON_IGNORE`d).

## Build & Run

Everything ROS runs in Docker (`Dockerfile`, `docker-compose.yml`, `docker/entrypoint.sh`). The image is `ros:foxy-ros-base` (arm64), and the `colcon build` happens at image build time.

```bash
docker compose build
docker compose up                 # base.launch.py startup_mode:=mapping (drivers, teleop, odom, web UI + a mapping run)
docker compose run --rm dev       # shell with src/ bind-mounted; build/ install/ log/ land in the repo root
# inside the dev container
colcon build && source install/setup.bash
ros2 launch bentobot_bringup base.launch.py            # always-on part; startup_mode:=idle|mapping, web:=false
ros2 launch bentobot_bringup mapping.launch.py         # slam_toolbox (normally started by bentobot_web)
ros2 launch bentobot_bringup localization.launch.py map:=/bentobot_ws/data/maps/<id>/map.yaml
ros2 launch bentobot_bringup teleop.launch.py          # base + mapping, outside the web UI's mode manager
# frontend (on any machine with Node; the Docker build does this in a node stage)
cd frontend && npm ci && npm run build                 # BENTOBOT_URL=http://<robot>:8080 npm run dev for a live-reload UI
ros2 launch bentobot_description bentobot_description.launch.py
ros2 launch bentobot_description gazebo_world.launch.py   # needs ros-foxy-gazebo-ros-pkgs, which is not in the image
```

- **Launch split.** `base.launch.py` is the always-on part: `rosbridge_server` (port 9090, Foxglove "Rosbridge" connection), the `base_link → base_laser` static TF, `joy_linux` (autorepeat 20 Hz, `respawn`), teleop, MCU bridge, the LD06 lidar (+ scan resampler), odom EKF and `bentobot_web` (`respawn`). `mapping.launch.py` (slam_toolbox async) and `localization.launch.py` (`nav2_map_server` + `nav2_amcl` + lifecycle manager, `map:=` a map YAML) are **modes** that `bentobot_web` starts/stops as subprocesses, one at a time. `teleop.launch.py` = base + mapping; the web UI then shows an "external" mapping run it can save but not stop.
- **Web UI** — `http://<robot>:8080` (`BENTOBOT_WEB_PORT`), bound to the IP of `BENTOBOT_WEB_IFACE` (default `wlan0`; an IP such as `0.0.0.0` also works), set in `docker-compose.yml`. No auth (trusted LAN).
- **Persistent data** lives in `./data` on the host → `/bentobot_ws/data`: `maps/<id>/` (`map.pgm`/`map.yaml`, `posegraph.posegraph`/`.data`, `thumb.png`, `meta.json`) and `controllers.json` (axis profile per controller). `/var/run/dbus` is mounted for the host's BlueZ.
- **DDS runs over UDP only** (`docker/fastdds_udp.xml`, `FASTRTPS_DEFAULT_PROFILES_FILE` in the image). With Foxy's Fast-DDS 2.1 shared-memory transport, processes started later (slam_toolbox/AMCL from the web UI, or a second run after a stop) often never received `/tf` from `bentobot_odom`, so slam_toolbox made no map. Keep this when changing the Docker setup; `ipc: host` is still set. Compose also sets `init: true` and `stop_signal: SIGINT` so `docker compose down` shuts launch down cleanly instead of SIGKILLing it after 10 s.
- Foxy is EOL. The base image's apt source is the frozen `snapshots.ros.org/foxy/final`. Some packages were never released for Foxy (`foxglove_bridge`, hence rosbridge). `joy_linux` is used instead of `joy` because Foxy's `joy` 3.x is SDL2-based and can number axes differently.
- The LD06 driver (`ldlidar_stl_ros2`, LDRobot) is cloned at a pinned commit (`LDLIDAR_COMMIT` in the Dockerfile) into a separate underlay workspace, `/opt/ldlidar_ws`, so bind-mounting `src/` doesn't hide it.
- To add an apt dependency, add it to the Dockerfile's apt list and to the package's `package.xml`. rosdep isn't used.
- There are no automated tests or linters. The frontend build runs `tsc --noEmit` (strict) before `vite build`.

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
                                                         bentobot_odom (EKF) --/odom + TF odom->base_link--> mode: slam_toolbox (mapping)
                                                                                                                or map_server + amcl (localization)
browser <--HTTP + /ws (port 8080)--> bentobot_web (rclpy + aiohttp) --subscribes /scan /mcu_info /odom /joy /mcu_cmd /map /amcl_pose, TF
                                          |  calls /bentobot_teleop/set_enabled, set_parameters; /slam_toolbox/serialize_map; publishes /initialpose
                                          |  runs `ros2 launch bentobot_bringup {mapping,localization}.launch.py` as a process group
                                          `--> host BlueZ over D-Bus (dbus-next)
```

- **Serial protocol (bridge ↔ firmware)** — plain text lines. Host → MCU: `"<lin_vel> <ang_vel>\n"`. MCU → host: `"<left_vel> <right_vel> <imu_gyro_z>\n"` at ~100 Hz. Changing either side requires changing `bentobot_mcu_bridge/mcu_bridge_node.py` and `full_sensor_suite.cpp` together. The bridge's `serial_port` parameter defaults to the Teensy's `/dev/serial/by-id/usb-Teensyduino_...` path. A background thread does the blocking serial reads, and the executor handles `mcu_cmd`.
- **Firmware (`full_sensor_suite`)** — quadrature encoder ISRs → per-wheel `MotorKalmanFilter` (velocity estimate) → per-wheel `MotorPID` (PID + exponential feed-forward `FF_A`, `FF_B`). It converts (v, w) to wheel velocities itself. The feed-forward constants come from `single_motor_feed_forward_calculator.py` plus the `*_test.csv` calibration data in `src/firmware/`. The other firmware folders are single-purpose bring-up/test sketches. `lib/MotorKalmanFilter` and `lib/MotorPID` are **copied** (not shared) between `full_sensor_suite` and `single_motor_calibration`, so keep them in sync.
- **Odometry (`bentobot_odom`)** — `Bentobot::KalmanFilter` is an EKF with a 5-state `[x, y, θ, v, w]`, a Runge-Kutta motion model, and two measurement updates: wheel odom (v, w) and IMU (w). It is ROS-agnostic, pure Eigen. Noise params are declared on the `bentobot_odom` node and loaded from `bentobot_bringup/config/bentobot_config.yaml` (the code has its own defaults). It publishes `/odom` (pose and the EKF's v, w as twist) and the dynamic `odom → base_link` TF.
- **Lidar** — the LD06 driver publishes `/scan_raw`, and `bentobot_laser`'s `scan_resampler_node` re-bins it to a fixed `beam_count` (450) on `/scan`. The driver sizes each scan by the points it received (~453–456), and slam_toolbox drops scans whose size changes.
- **Mapping** — `slam_toolbox` replaces gmapping. Its config is `bentobot_bringup/config/slam_toolbox_config.yaml` and uses TF only, not `/odom`.
- **Localization** — AMCL on the saved/edited **PGM** (`config/amcl_config.yaml`, `robot_model_type: differential` is the Foxy spelling). Map edits in the web editor only affect this path; the slam_toolbox pose graph is saved alongside (for a future slam_toolbox localization or continued mapping) but never edited. Saved YAMLs use `free_thresh: 0.196`, as ROS 1 map_saver did: with 0.25, map_server reads unknown pixels (205) as free.
- **Teleop (`bentobot_teleop`)** — axis indices, scales, inversion, deadzone and `joy_timeout` are parameters (defaults in `bentobot_config.yaml`, changeable at runtime). `~/set_enabled` (std_srvs/SetBool) pauses teleop: it publishes one zero Twist and ignores `/joy`; the state is latched on `~/enabled`. The web STOP button uses it (and also publishes a zero `/mcu_cmd` itself). If `/joy` goes quiet for `joy_timeout` while moving, teleop sends a zero Twist (joy_linux autorepeat keeps `/joy` flowing while the controller is connected).
- **Web backend (`bentobot_web`)** — `main.py` runs rclpy in a thread and aiohttp on asyncio. `ros_iface.py` (subscriptions, rate/staleness tracking, tf2 `map → base_link` at 10 Hz with an `odom` fallback, service clients), `modes.py` (ModeManager; `MODES` registry — add a navigation mode there; SIGINT to `ros2 launch`, then SIGTERM/SIGKILL to the group; PR_SET_PDEATHSIG; pid file with start time for crash cleanup; detects runs started outside the UI after a 30 s grace), `web.py` (HTTP API under `/api`, `/ws`, static files from `/opt/bentobot_web/static`), `maps.py` + `mapimg.py` (map library, PGM/YAML written from the last `/map`, no map_saver), `bluetooth.py` (BlueZ: discovery, pair with a NoInputNoOutput agent, trust, connect, remove), `controllers.py`, `system.py` (psutil, `/sys/class/thermal`, `/sys/class/input/js0`).
- **Live data** goes over the backend websocket, not rosbridge: status JSON 2 Hz, telemetry JSON 10 Hz, scan as binary float32 ranges 5 Hz, path deltas, and `/map` as an 8-bit PNG only when it changes (≤ 1/s). Binary frames are `<uint32 LE header length><JSON header><payload>`. During a localization run the map comes from the PGM on disk.
- **Frontend (`frontend/`)** — Preact + Vite + TypeScript, ~22 kB gzipped JS, hash routes `#/` status, `#/run`, `#/maps`, `#/edit/<id>`, `#/controller`. Maps are drawn on canvases (`components/MapView.tsx`, `pages/Editor.tsx`, shared pan/zoom/pinch in `viewport.ts`). Built in the Dockerfile's node stage; only `dist/` goes into the image.
- **Shared physical constants** are duplicated across layers: `WHEEL_BASE_LENGTH = 0.16` (firmware and `odom_node.cpp`), wheel diameter 0.068 and 680 ticks/rev (firmware). The `base_link → base_laser` static TF is in `bentobot_bringup/launch/description.launch.py`.
- `bentobot_interfaces` defines the `MCUInfo` message (`left_vel`, `right_vel`, `imu_ang_vel` + header). `bentobot_mcu_bridge` and `bentobot_odom` depend on it.
- The Gazebo sim (`bentobot_description/urdf/bentobot.gazebo`) uses the ROS 2 `gazebo_ros` diff-drive and ray-sensor plugins on `/cmd_vel`, `/odom`, `/scan`. Note that it uses `robot_footprint`/`lidar` frames, not the real robot's `base_link`/`base_laser`.
