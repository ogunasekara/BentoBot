#!/bin/bash
# Sources the ROS 2 workspaces. Runs as the container entrypoint, and is also
# sourced from .bashrc so `docker compose exec` shells get the same environment.

source /opt/ros/foxy/setup.bash
source /opt/ldlidar_ws/install/setup.bash

# missing in the dev container until the first colcon build
if [ -f /bentobot_ws/install/setup.bash ]; then
    source /bentobot_ws/install/setup.bash
fi

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    exec "$@"
fi
