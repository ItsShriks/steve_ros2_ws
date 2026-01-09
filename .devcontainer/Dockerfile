# Define base image args before FROM
ARG DOCKER_REPO=osrf/ros
ARG ROS_DISTRO=humble
ARG IMAGE_SUFFIX=-desktop-full

FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX

# Define user args after FROM (defaults to match typical host)
ARG USERNAME=$USER
ARG USER_UID=1000
ARG USER_GID=1000

# Make apt non-interactive
ARG DEBIAN_FRONTEND=noninteractive

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*
# Install additonal packages - add any that you need
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends python3-pip python-is-python3 ssh neovim git python3-tqdm python3-requests \
    ros-$ROS_DISTRO-libg2o ros-$ROS_DISTRO-robotiq-description ros-$ROS_DISTRO-dwb-critics ros-$ROS_DISTRO-nav2-core ros-$ROS_DISTRO-nav2-costmap-2d \
    ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-nav2-util ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-librealsense2 \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-launch-pytest ros-$ROS_DISTRO-moveit-kinematics ros-$ROS_DISTRO-moveit-planners-ompl \
    ros-$ROS_DISTRO-moveit-ros-move-group ros-$ROS_DISTRO-moveit-ros-visualization ros-$ROS_DISTRO-moveit-servo ros-$ROS_DISTRO-moveit-simple-controller-manager \
    ros-$ROS_DISTRO-ur-description ros-$ROS_DISTRO-warehouse-ros-sqlite ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-common \
    ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-phidgets-spatial \
     ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers \
     ros-$ROS_DISTRO-gazebo-ros2-control ros-$ROS_DISTRO-gazebo-plugins ros-$ROS_DISTRO-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*
ENV SHELL=/bin/bash

# Source ROS environment automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
#RUN echo "source /home/$USERNAME/steve_ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
# Set the default user
USER $USERNAME
CMD ["/bin/bash"]
