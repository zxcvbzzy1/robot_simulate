FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
RUN sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|; s|Types: deb deb-src|Types: deb|" /etc/apt/sources.list.d/ros2.sources \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-planners-ompl ros-jazzy-robot-state-publisher \
    ros-jazzy-control-msgs ros-jazzy-trajectory-msgs ros-jazzy-visualization-msgs \
    python3-numpy python3-scipy python3-yaml \
    && rm -rf /var/lib/apt/lists/*
COPY entrypoint.sh /weld-entrypoint.sh
ENTRYPOINT ["/bin/bash", "/weld-entrypoint.sh"]
