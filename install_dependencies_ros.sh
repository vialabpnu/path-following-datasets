#!/bin/bash

# Update the package list
sudo apt-get update

# Install the dependencies
sudo apt-get install -y \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    tmux \
    ros-melodic-catkin \
    ros-melodic-message-to-tf \
    ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-laser-proc \
    ros-melodic-rgbd-launch \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino \
    ros-melodic-rosserial-python \
    ros-melodic-rosserial-server \
    ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-urdf \
    ros-melodic-xacro \
    ros-melodic-compressed-image-transport \
    ros-melodic-rqt-image-view \
    ros-melodic-gmapping \
    ros-melodic-navigation \
    ros-melodic-interactive-markers \
    ros-melodic-rosbridge-suite \
    ros-melodic-tf2-web-republisher \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-teleop-tools \
    ros-melodic-joy \
    ros-melodic-mpc-local-planner \
    ros-melodic-ompl \
    ros-melodic-moveit-ros \
    ros-melodic-pr2-description \
    ros-melodic-velodyne-description \
    ros-melodic-twist-mux \
    ros-melodic-ros-control \
    ros-melodic-joy-teleop \
    ros-melodic-velocity-controllers \
    ros-melodic-effort-controllers \
    ros-melodic-ackermann-msgs \
    ros-melodic-robot-state-publisher \
    ros-melodic-joint-state-controller \
    ros-melodic-joint-state-controller-dbgsym \
    ros-melodic-nav-core \
    python-pip \
    wget

# Source the ROS setup script
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Install Python 2 dependencies
pip2 install -r py2_requirements_ros.txt

# Create conda environment based on the provided environment.yml file
conda env create -f mpc_environment.yml

# Check if the conda for mpc is created successfully
if conda env list | grep -q mpc-gen; then
    echo "Conda MPC environment created successfully"
else
    echo "Conda MPC environment not created successfully"
fi
