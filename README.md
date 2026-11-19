# BentoBot

## Overview

Bentobot is a mobile robotics platform to test custom C++ ROS packages for mobile robot mapping, localization, and path planning. The two-wheel differential drive chassis was designed using [OnShape](https://www.onshape.com/en/) and 3D printed on the Ender 3 V2. All design and software artifacts will ultimately be included in this repository. The robot was designed to utilize off the shelf components that can be purchased online.

## Software Dependencies

- ROS 2 Foxy, run in Docker (see below)
- Eigen3

## Running on the Robot (Jetson Nano)

The ROS 2 stack runs in a Docker container (`ros:foxy-ros-base`, Ubuntu 20.04 arm64), so the Jetson can stay on its stock JetPack 4.x / Ubuntu 18.04 image.

One-time setup on the Jetson:

1. JetPack ships Docker. Add yourself to the `docker` group: `sudo usermod -aG docker $USER`, then log out and back in.
2. Install the [Docker Compose v2 plugin](https://docs.docker.com/compose/install/linux/#install-the-plugin-manually) for `aarch64`. The `docker-compose` in the Ubuntu 18.04 apt repo is too old for `docker-compose.yml`.

Build and run:

```
docker compose build                 # builds the image, including colcon build
docker compose up                    # drivers, teleop, odometry and the web UI; starts a mapping run
docker compose run --rm dev          # dev shell with src/ bind-mounted; run `colcon build` inside
```

The container uses the host network and has access to all of `/dev`, so the Teensy, LD06 and joystick don't need to be listed individually.

The image can also be built on an arm64 machine (e.g. an Apple Silicon Mac) and copied over with `docker save bentobot:foxy | ssh <robot> docker load`.

To view the robot in [Foxglove Studio](https://foxglove.dev/), open a **Rosbridge** connection to `ws://<robot>:9090`. `foxglove_bridge` was never released for Foxy, so the robot runs `rosbridge_server` instead.

## Web Control Panel

`docker compose up` also starts the web UI at `http://<robot>:8080` (e.g. `http://bentobot.local:8080` if the Jetson runs avahi). Open it from a phone or laptop on the same network. It has:

- **Status**: mode, node liveness, topic rates (stale after ~1 s), wheel/IMU/odometry values, the controller, and Jetson CPU, RAM, temperatures and disk space. The **STOP** button pauses teleop and zeroes the motors until you press **Resume**.
- **Run**: start a mapping run (slam_toolbox) or a localization run (AMCL) on a saved map. The live map, laser scan, robot pose and driven path are shown. Mapping runs are saved under a name (slam_toolbox pose graph + PGM/YAML) or discarded. Localization runs show AMCL's confidence and let you set the initial pose by pressing and dragging on the map.
- **Maps**: saved maps with thumbnails; localize, edit, rename, download (zip) or delete.
- **Editor**: paint free/occupied/unknown cells, draw virtual walls and rectangles, undo/redo, and save as a new map or overwrite. Edits change what AMCL localizes on, not the slam_toolbox pose graph.
- **Controller**: pair, connect and forget Bluetooth gamepads, and pick the stick axes for driving (saved per controller).

Settings (environment variables for `docker compose`, e.g. in a `.env` file next to `docker-compose.yml`):

| Variable | Default | |
|---|---|---|
| `BENTOBOT_WEB_PORT` | `8080` | HTTP port |
| `BENTOBOT_WEB_IFACE` | `wlan0` | interface to listen on (`eth0`, or `0.0.0.0` for all) |

Saved maps and controller profiles are stored in `./data` on the Jetson, so they survive image rebuilds. There is no login, so only run it on a trusted network.

### One-time Bluetooth setup on the Jetson

The web UI talks to the Jetson's own BlueZ over D-Bus (`/var/run/dbus` is mounted into the container), so Bluetooth must be running on the host:

```
sudo systemctl enable --now bluetooth
rfkill list                          # if Bluetooth shows "Soft blocked: yes":
sudo rfkill unblock bluetooth
```

Some gamepads (DualShock 4, 8BitDo and many generic pads) won't connect while BlueZ only accepts bonded HID devices. If a pad pairs but never shows up as `/dev/input/js0`, edit `/etc/bluetooth/input.conf` and set

```
ClassicBondedOnly=false
```

then `sudo systemctl restart bluetooth`. Ubuntu 18.04 (JetPack 4.x) ships BlueZ 5.48 with `ClassicBondedOnly` backported from a security fix, so this setting applies there too. On BlueZ 5.56 and newer, `UserspaceHID=true` in the same file also helps some BLE pads. Check the version with `bluetoothd -v`.

## Remote Development Setup

I currently use the [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) VSCode extension to remotely develop on the robot. To use this, ensure SSH is setup on the robot and utilize Remote - SSH's connect to host to connect to the robot. Build and run the code inside the `dev` container described above.

## PlatformIO Setup

I am utilizing PlatformIO as a convenient way to upload firmware to the Teensy 4.0 onboard BentoBot. All of the firmware is contained in the `src/firmware` folder. To upload 
