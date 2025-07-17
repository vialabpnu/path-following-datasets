#!/bin/bash

# Update the package list
sudo apt-get update

# Install the dependencies
sudo apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    tmux \
    ros-noetic-catkin \
    ros-noetic-message-to-tf \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-laser-proc \
    ros-noetic-rgbd-launch \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python \
    ros-noetic-rosserial-server \
    ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-compressed-image-transport \
    ros-noetic-rqt-image-view \
    ros-noetic-gmapping \
    ros-noetic-navigation \
    ros-noetic-interactive-markers \
    ros-noetic-rosbridge-suite \
    ros-noetic-tf2-web-republisher \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-teleop-tools \
    ros-noetic-joy \
    ros-noetic-mpc-local-planner \
    ros-noetic-ompl \
    ros-noetic-moveit-ros \
    ros-noetic-pr2-description \
    ros-noetic-velodyne-description \
    ros-noetic-twist-mux \
    ros-noetic-ros-control \
    ros-noetic-joy-teleop \
    ros-noetic-velocity-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-ackermann-msgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-controller-dbgsym \
    ros-noetic-nav-core \
    python3-pip \
    wget

# Source the ROS setup script
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install Python 3 dependencies
pip3 install -r py3_requirements_ros_noetic.txt

# Create conda environment based on the provided environment.yml file
conda env create -f mpc_environment.yml

# Check if the conda for mpc is created successfully
if conda env list | grep -q mpc-gen; then
    echo "Conda MPC environment created successfully"
else
    echo "Conda MPC environment not created successfully"
fi 