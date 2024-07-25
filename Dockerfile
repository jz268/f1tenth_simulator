FROM ros:melodic

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    rm -rf /var/cache/apt/archives/lock && \
    DEBIAN_FRONTEND=noninteractive \
    apt-get install --no-install-recommends --assume-yes \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-ackermann-msgs \
    ros-melodic-joy \
    ros-melodic-map-server \
    ros-melodic-interactive-markers \
    ros-melodic-xacro \
    ros-melodic-rviz \
    ros-melodic-robot-state-publisher \
    # ros-melodic-catkin python-catkin-tools \
    x11-xserver-utils \
    curl wget zsh vim
