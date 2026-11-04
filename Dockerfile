FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        libeigen3-dev \
        python3-serial \
        ros-foxy-joint-state-publisher \
        ros-foxy-joy-linux \
        ros-foxy-nav-msgs \
        ros-foxy-robot-state-publisher \
        ros-foxy-rosbridge-server \
        ros-foxy-slam-toolbox \
        ros-foxy-tf2-ros \
        ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

# LD06 lidar driver. It is built in its own underlay workspace so that
# bind-mounting src/ over /bentobot_ws/src during development doesn't hide it.
ARG LDLIDAR_COMMIT=bf668a89baf722a787dadc442860dcbf33a82f5a
RUN git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /opt/ldlidar_ws/src/ldlidar_stl_ros2 \
    && git -C /opt/ldlidar_ws/src/ldlidar_stl_ros2 checkout ${LDLIDAR_COMMIT} \
    && cd /opt/ldlidar_ws \
    && source /opt/ros/foxy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

WORKDIR /bentobot_ws
COPY src/ src/
RUN source /opt/ros/foxy/setup.bash \
    && source /opt/ldlidar_ws/install/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo 'source /entrypoint.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "bentobot_bringup", "teleop.launch.py"]
