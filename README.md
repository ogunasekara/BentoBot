# BentoBot

## Overview

Bentobot is a mobile robotics platform to test custom C++ ROS packages for mobile robot mapping, localization, and path planning. The two-wheel differential drive chassis was designed using [OnShape](https://www.onshape.com/en/) and 3D printed on the Ender 3 V2. All design and software artifacts will ultimately be included in this repository. The robot was designed to utilize off the shelf components that can be purchased online.

## Software Dependencies

- ROS Melodic
- Eigen3

## Remote Development Setup

I currently use the [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) VSCode extension to remotely develop on the robot. To use this, ensure SSH is setup on the robot and utilize Remote - SSH's connect to host to connect to the robot.

To ensure that C++ libraries show up properly in the remote development environment, install the [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack) on VSCode.

In the auto-generated `.vscode/c_pp_properties.json` file, ensure the includePath has the following directories:

```
${workspaceFolder}/**",
"<repository path>/devel/include/**",
"/opt/ros/melodic/include/**",
"/usr/include/**"
```

## PlatformIO Setup

I am utilizing PlatformIO as a convenient way to upload firmware to the Teensy 4.0 onboard BentoBot. All of the firmware is contained in the `src/firmware` folder. To upload 
