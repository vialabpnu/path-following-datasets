# Use a base image with ROS Melodic pre-installed
FROM ros:melodic-ros-base

# Set arguments for a non-root user
ARG USER=ubuntu
ARG UID=1000
ARG GID=1000

# Create user and group
RUN groupadd -g $GID $USER && \
    useradd -u $UID -g $GID -m $USER

# Install system dependencies as root
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-backports.functools-lru-cache \
    python-wstool \
    python-tk \
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

# Switch to non-root user and set working directory
USER $USER
WORKDIR /home/$USER

# Clone the repository which contains all necessary config files
RUN git clone https://github.com/vialabpnu/path-following-datasets.git /home/$USER/path-following-datasets

# --- FIX START ---
# Move files from the cloned repository to their required locations
# User-level files go into the home directory
RUN cp /home/$USER/path-following-datasets/py2_requirements_ros_melodic.txt /home/$USER/ && \
    cp /home/$USER/path-following-datasets/mpc_environment.yml /home/$USER/ && \
    cp /home/$USER/path-following-datasets/install_dependencies_ros_melodic.sh /home/$USER/

# Switch to root to place system-level config files
USER root
RUN cp /home/$USER/path-following-datasets/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
# Switch back to the non-root user
USER $USER
# --- FIX END ---

# Install Python 2 dependencies
RUN python2 -m pip install --upgrade pip==20.3.4 && \
    python2 -m pip install -r py2_requirements_ros_melodic.txt

# Verify package installation
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

# Initialize rosdep as root
USER root
RUN rosdep init || true && \
    rosdep update
USER $USER

# Run the dependency installation script
RUN chmod +x /home/$USER/install_dependencies_ros_melodic.sh && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && /home/$USER/install_dependencies_ros_melodic.sh"

# Activate conda environment for all subsequent commands
SHELL ["conda", "run", "-n", "mpc-gen", "/bin/bash", "-c"]

# Build the catkin workspace
RUN source /opt/ros/melodic/setup.bash && \
    cd /home/ubuntu/path-following-datasets/examples/car_ws && \
    catkin build

# Set the final container entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /home/ubuntu/miniconda/etc/profile.d/conda.sh && conda activate mpc-gen && source /opt/ros/melodic/setup.bash && source /home/ubuntu/path-following-datasets/examples/car_ws/devel/setup.bash && exec \"$@\"", "bash"]

# Default command to run
CMD ["/bin/bash"]
