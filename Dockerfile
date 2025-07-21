# Use a base image with ROS Melodic pre-installed
FROM ros:melodic-ros-base

# Set the user to a non-root user (e.g., the default user in the ros image)
ARG USER=ubuntu
ARG UID=1000
ARG GID=1000

# Create a user and group with the specified UID and GID
RUN groupadd -g $GID $USER && \
    useradd -u $UID -g $GID -m $USER

# Set working directory to the user's home directory
USER $USER
WORKDIR /home/$USER

# Install system dependencies (as the non-root user)
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    tmux \
    supervisor \
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
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-plugins \
    ros-melodic-gazebo-ros-control \
    python-pip \
    wget \
    bzip2 \
    && rm -rf /var/lib/apt/lists/*
USER $USER

# Clone the repository to the correct location
RUN git clone https://github.com/vialabpnu/path-following-datasets.git /home/$USER/path-following-datasets

# (Optional) Debug: List contents to verify
RUN ls -l /home/$USER/path-following-datasets/examples

# Copy necessary files for dependencies (adjust paths if needed)
COPY --chown=$USER:$USER py2_requirements_ros_melodic.txt /home/$USER/
COPY --chown=$USER:$USER mpc_environment.yml /home/$USER/
COPY --chown=$USER:$USER install_dependencies_ros_melodic.sh /home/$USER/
COPY --chown=$USER:$USER supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Install Python 2 dependencies using pip2
RUN pip2 install --upgrade pip==20.3.4 \
    && pip2 install -r py2_requirements_ros_melodic.txt

# Temporarily switch to root to install the package globally
USER root
RUN python2 -m pip install backports.functools-lru-cache \
    && python2 -c "from backports.functools_lru_cache import lru_cache"

# Switch back to the default user
USER ubuntu

# Now, we can verify that the 'ubuntu' user can find the globally installed package
RUN python2 -c "from backports.functools_lru_cache import lru_cache"

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p /home/$USER/miniconda && \
    rm miniconda.sh

# Add conda to PATH
ENV PATH="/home/$USER/miniconda/bin:$PATH"

# Accept Conda Terms of Service for required channels
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# Create conda environment
RUN conda env create -f mpc_environment.yml

# Initialize rosdep
USER root
RUN rosdep init || true && \
    rosdep update
USER $USER

# Run the dependency installation script
RUN chmod +x /home/$USER/install_dependencies_ros_melodic.sh && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && /home/$USER/install_dependencies_ros_melodic.sh"

# Activate conda environment for later use
SHELL ["conda", "run", "-n", "mpc-gen", "/bin/bash"]

# Source ROS setup (Optional), It should automatically done as we used the ros docker image
# RUN echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc

# You can add further instructions here, e.g., to build your ROS workspace if needed
# Build the catkin workspace
RUN ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && cd /home/ubuntu/path-following-datasets/examples/car_ws && catkin build"]

# This runs every time you start the container with 'docker run'.
ENTRYPOINT ["/bin/bash", "-c", "source /home/ubuntu/miniconda/etc/profile.d/conda.sh && conda activate mpc-gen && source /opt/ros/melodic/setup.bash && exec \"$@\"", "bash"]

# This is the default command that runs if you don't provide one.
# It will be executed by the ENTRYPOINT, dropping you into an interactive shell.
CMD ["/bin/bash"]