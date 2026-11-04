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
docker compose up                    # runs teleop.launch.py
docker compose run --rm dev          # dev shell with src/ bind-mounted; run `colcon build` inside
```

The container uses the host network and has access to all of `/dev`, so the Teensy, LD06 and joystick don't need to be listed individually.

The image can also be built on an arm64 machine (e.g. an Apple Silicon Mac) and copied over with `docker save bentobot:foxy | ssh <robot> docker load`.

To view the robot in [Foxglove Studio](https://foxglove.dev/), open a **Rosbridge** connection to `ws://<robot>:9090`. `foxglove_bridge` was never released for Foxy, so the robot runs `rosbridge_server` instead.

## Remote Development Setup

I currently use the [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) VSCode extension to remotely develop on the robot. To use this, ensure SSH is setup on the robot and utilize Remote - SSH's connect to host to connect to the robot. Build and run the code inside the `dev` container described above.

## PlatformIO Setup

I am utilizing PlatformIO as a convenient way to upload firmware to the Teensy 4.0 onboard BentoBot. All of the firmware is contained in the `src/firmware` folder. To upload 
