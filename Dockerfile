# ---- frontend: built with Node, only dist/ is copied into the robot image ----
# Runs on the build machine's native platform; the output is plain static files.
FROM --platform=$BUILDPLATFORM node:20-bookworm-slim AS frontend
WORKDIR /frontend
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY frontend/ ./
RUN npm run build

# ---- ROS dependencies (apt, pip, LD06 driver) ----
FROM ros:foxy-ros-base AS deps

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        libeigen3-dev \
        python3-aiohttp \
        python3-numpy \
        python3-pil \
        python3-pip \
        python3-psutil \
        python3-serial \
        ros-foxy-joint-state-publisher \
        ros-foxy-joy-linux \
        ros-foxy-nav-msgs \
        ros-foxy-nav2-amcl \
        ros-foxy-nav2-lifecycle-manager \
        ros-foxy-nav2-map-server \
        ros-foxy-nav2-msgs \
        ros-foxy-robot-state-publisher \
        ros-foxy-rosbridge-server \
        ros-foxy-slam-toolbox \
        ros-foxy-std-srvs \
        ros-foxy-tf2-ros \
        ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

# BlueZ D-Bus client for bentobot_web (pure Python, no apt package on focal)
RUN pip3 install --no-cache-dir dbus-next==0.2.3

# LD06 lidar driver. It is built in its own underlay workspace so that
# bind-mounting src/ over /bentobot_ws/src during development doesn't hide it.
ARG LDLIDAR_COMMIT=bf668a89baf722a787dadc442860dcbf33a82f5a
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /opt/ldlidar_ws/src/ldlidar_stl_ros2 \
    && git -C /opt/ldlidar_ws/src/ldlidar_stl_ros2 checkout ${LDLIDAR_COMMIT} \
    && cd /opt/ldlidar_ws \
    && source /opt/ros/foxy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ---- robot image ----
FROM deps AS runtime

WORKDIR /bentobot_ws
COPY src/ src/
RUN source /opt/ros/foxy/setup.bash \
    && source /opt/ldlidar_ws/install/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# web UI, served by bentobot_web (static_dir parameter)
COPY --from=frontend /frontend/dist /opt/bentobot_web/static

# DDS over UDP only: Fast-DDS's shared-memory transport drops data to processes
# that join late (slam_toolbox / AMCL started by the web UI); see the XML
COPY docker/fastdds_udp.xml /opt/bentobot/fastdds_udp.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/bentobot/fastdds_udp.xml

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo 'source /entrypoint.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bentobot_bringup", "base.launch.py", "startup_mode:=mapping"]
