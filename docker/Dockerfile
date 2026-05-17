FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ARG UBUNTU_MIRROR="http://archive.ubuntu.com/ubuntu"
ARG UBUNTU_SECURITY_MIRROR="http://security.ubuntu.com/ubuntu"
ARG ROS_MIRROR="http://packages.ros.org/ros2/ubuntu"
ARG ROSDEP_BASE_URL="https://raw.githubusercontent.com/ros/rosdistro/master"
ENV ROSDEP_BASE_URL=${ROSDEP_BASE_URL}
ARG ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml"
ENV ROSDISTRO_INDEX_URL=${ROSDISTRO_INDEX_URL}

WORKDIR /ws
RUN sed -i "s@http://archive.ubuntu.com/ubuntu@${UBUNTU_MIRROR}@g" /etc/apt/sources.list && \
    sed -i "s@http://security.ubuntu.com/ubuntu@${UBUNTU_SECURITY_MIRROR}@g" /etc/apt/sources.list && \
    if [ -f /etc/apt/sources.list.d/ros2.list ]; then \
        sed -i "s@https\?://packages.ros.org/ros2/ubuntu@${ROS_MIRROR}@g" /etc/apt/sources.list.d/ros2.list; \
    fi && \
    apt-get update && apt-get install -y --no-install-recommends python3-rosdep python3-colcon-common-extensions

WORKDIR /ws/src
# Copy only dependency manifests first for better cache reuse.
COPY robot_bringup/package.xml robot_bringup/package.xml
COPY robot_description/package.xml robot_description/package.xml
COPY robot_hardware_hal/package.xml robot_hardware_hal/package.xml
COPY robot_simulation/package.xml robot_simulation/package.xml

RUN \
    (rosdep init || true) && \
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        sed -i "s@https://raw.githubusercontent.com/ros/rosdistro/master@${ROSDEP_BASE_URL}@g" /etc/ros/rosdep/sources.list.d/20-default.list; \
    fi && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths /ws/src --ignore-src -r -y

COPY . /ws/src

WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch robot_bringup system_sim.launch.py"]