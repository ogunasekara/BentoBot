# Task: build a web control panel for BentoBot, served from the robot

Build a web frontend for BentoBot that is hosted on the Jetson Nano and can be opened from any
browser on the same LAN (e.g. http://bentobot.local:8080). Read CLAUDE.md first. The robot runs
ROS 2 Foxy in Docker (`network_mode: host`, `privileged`, `/dev` mounted), and rosbridge is
already running on ws://<robot>:9090.

## What the UI must do

1. **Robot status dashboard (live)**
   - Which mode the robot is in: idle / mapping run / localization run. Also the active map's name.
   - Liveness of each node: joy, bentobot_teleop, bentobot_mcu_bridge, LD06 driver, scan_resampler,
     bentobot_odom, slam_toolbox / localization. Show message rates for `/scan`, `/mcu_info`,
     `/odom` and `/joy`. Flag a topic as stale if it goes quiet for more than about 1 s.
   - Live values: wheel velocities and IMU gyro z from `/mcu_info`, and v/w plus pose from `/odom`.
     Show the commanded `/mcu_cmd`.
   - Controller state: whether `/dev/input/js0` exists, which Bluetooth controller is connected,
     and live stick values from `/joy`.
   - Jetson health: CPU and RAM use, temperatures from `/sys/class/thermal`, disk free space for
     map storage.
   - A software **STOP** button that halts the robot and pauses teleop. Note that
     `bentobot_teleop` republishes on every `/joy` message, so publishing a single zero Twist is
     not enough. Add an enable/disable mechanism to the teleop node, such as a service or
     parameter, and use it.

2. **Bluetooth controller management**
   - Scan for devices, then pair, trust, connect, disconnect and forget them. List both paired
     and nearby devices.
   - Use the host's BlueZ through D-Bus (`bluetoothctl` or `dbus-next`/`pydbus`). The container
     probably needs `/var/run/dbus` bind-mounted. Check whether the host's bluetoothd is running.
     Document any host setup the user has to do once, such as enabling bluetooth.service or
     setting `ClassicBondedOnly`/`UserspaceHID` for some gamepads.
   - After a controller connects, confirm it shows up as `/dev/input/js0` and that `joy_linux`
     picks it up. Restart the joy node if it has to be restarted.
   - Axis mapping: `bentobot_teleop` hard-codes `axes[1]` (linear) and `axes[2]` (angular), and
     different controllers number their axes differently. Make the axis indices and scales
     parameters of the teleop node. Provide a simple calibration page that shows live `/joy`
     values so I can choose the axes. Save the choice for each controller.

3. **Teleoperated exploration (mapping) runs**
   - Start a run: launch slam_toolbox in mapping mode. I drive with the controller while the live
     map builds in the browser.
   - Live view: the `/map` OccupancyGrid, the robot pose (TF `map → base_link`), the latest
     `/scan` points overlaid, and the path driven so far. Support pan and zoom, and work on a
     phone-sized screen.
   - Stop and save: enter a name, then save **both** the slam_toolbox serialized pose graph
     (`/slam_toolbox/serialize_map`) and an occupancy image (PGM + YAML). Save the image by
     writing it from the last `/map` message in the backend. Don't depend on `nav2_map_server`,
     which isn't installed. Also offer to discard the run.

4. **Map library**
   - List saved maps with a thumbnail, created date, size/resolution and run duration. Support
     rename, delete and download (zip).
   - Store maps in a host-mounted directory (e.g. `./maps` → `/bentobot_ws/maps`) so they survive
     container rebuilds.

5. **Map editor**
   - Open a saved map and paint free/occupied/unknown cells with a brush or rectangle tool.
     Support erasing artifacts, drawing virtual walls, undo/redo, and save as a new version or
     overwrite.
   - Important: slam_toolbox localization runs on the **serialized pose graph**, not the PGM, so
     editing the PGM has no effect on that path. Pick one of these before building it and tell me
     the trade-off:
     (a) localization runs use `nav2_amcl` + `nav2_map_server` on the edited PGM. This needs
         `ros-foxy-navigation2` packages from the snapshot (check they exist), so the edits
         actually matter. **I lean toward this one.**
     (b) localization uses slam_toolbox localization mode, and edits only affect the
         displayed/exported map.

6. **Localized runs on an existing map**
   - Pick a map and start a localization run. Show the map, the live pose, the scan overlay and
     localization confidence (AMCL covariance or equivalent).
   - Set an initial pose by click-and-drag on the map, published to `/initialpose`.
   - Driving is still teleop through the controller. There is no autonomous navigation yet. Keep
     the mode system extensible so navigation goals can be added later.

## Architecture requirements

- **Split the launch files.** Move the always-on parts of `teleop.launch.py` into a base bringup:
  rosbridge, description/static TF, joy_linux, teleop, MCU bridge, lidar + resampler, odom EKF.
  Create separate `mapping.launch.py` and `localization.launch.py` (taking a map argument) that
  the backend starts and stops. Keep `teleop.launch.py` working as base + mapping so
  `docker compose up` behaves as it does today.
- **Backend**: a new ROS package, e.g. `bentobot_web` (Python, rclpy, runs in the same container).
  It provides:
  - a small HTTP API (FastAPI/uvicorn or aiohttp; it must run on Python 3.8 / arm64) for modes,
    maps, bluetooth, system stats and the static frontend files
  - a process manager that starts/stops the mode launch files as subprocesses (process group,
    SIGINT then SIGTERM, cleanup on crash). Only one mode runs at a time.
  - live data either through rosbridge (roslibjs in the browser) or a backend websocket. Choose
    one and justify it. Large `/map` messages must not choke the Jetson or Wi-Fi, so throttle
    and/or compress them (rosbridge `throttle_rate`, PNG compression, or the backend sending
    diffs/PNG).
  - Foxy has no tf2_web_republisher, so compute the `map → base_link` pose in the backend (tf2
    buffer) and publish or stream it at about 10 Hz.
- **Frontend**: a single-page app built to static files. Build it in a multi-stage Docker stage
  (node image) and copy only the built `dist/` into the runtime image, so there is no Node on the
  Jetson at runtime. Keep the bundle light; this runs on a Nano and on phones. Draw maps on a
  canvas, not as SVG cells.
- **Docker**: add apt/pip dependencies to the Dockerfile (and `package.xml`; rosdep isn't used),
  add the maps volume, `/var/run/dbus`, and the web port. The web server should start as part of
  `docker compose up`.
- No auth is needed (trusted LAN), but bind to the LAN interface and make the port configurable.

## How to work

- Before writing code, give me a short plan: the package layout, the API endpoints, the
  live-data transport choice, and your answer to the map-editor decision in 5. Wait for my
  go-ahead.
- Build in this order: launch split + process manager → status dashboard → mapping run +
  save → map library → localization run → map editor → bluetooth.
- I can't test hardware from your session. For each step, tell me exactly what to run on the
  robot to verify it, and what I should see. Don't claim something works unless you ran it.
- Update CLAUDE.md with the new package, launch files, ports and data flow when done.
- Commit directly on the `ros2-foxy` branch (it holds the ROS 2 conversion; `main` doesn't).
  Don't commit to `main` and don't create a feature branch.
