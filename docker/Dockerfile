# Use ROS 2 Humble base image
FROM ros:humble

ARG ROS2_DISTRO=humble

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard \
    ros-humble-mavros \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    ros-humble-plotjuggler-ros \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rqt* \
    ros-humble-nmea-msgs \
    ros-humble-mavros-msgs \
    apt-transport-https \
    cmake \
    libasio-dev \
    libtinyxml2-dev \
    nano \
    vim \
    can-utils \
    iproute2 \
    net-tools \
    tmux \
    tmuxinator \
    && rm -rf /var/lib/apt/lists/*

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone
#RUN ip link set can0 up type can bitrate 500000

# Agent forwarding during docker build https://stackoverflow.com/questions/43418188/ssh-agent-forwarding-during-docker-build
# install ssh client and git
RUN sudo apt-get install openssh-client git

ENV DOCKER_BUILDKIT=1
RUN apt-get install -y openssh-client
ENV GIT_SSH_COMMAND="ssh -v" 
USER root

RUN --mount=type=ssh id=default \
    mkdir -p -m 0600 ~/.ssh/ && \
    ssh-keyscan github.com >> ~/.ssh/known_hosts


RUN apt-get update

# Livox SDK
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j
RUN sudo make install

# Set up a workspace
WORKDIR /root/scout_ws/src

# Clone the scout ROS 2 repository
RUN git clone --single-branch --branch humble https://github.com/westonrobot/scout_ros2.git
RUN git clone https://github.com/westonrobot/ugv_sdk.git
RUN apt-get install -y ros-humble-tf2-geometry-msgs

# Sensor fusion
#RUN --mount=type=ssh git clone --branch master git@github.com:larics/sensor_fusion.git
RUN --mount=type=ssh git clone --branch scout git@github.com:larics/uav_ros_estimators.git
RUN --mount=type=ssh git clone --branch ros2 git@github.com:larics/uav_ros_msgs.git
RUN --mount=type=ssh git clone --branch humble-dev git@github.com:larics/uav_ros_lib.git

# Xsens Movella packages
RUN --mount=type=ssh git clone --branch ros2 git@github.com:xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git

# Livox ros driver
RUN --mount=type=ssh git clone git@github.com:Ericsii/livox_ros_driver2.git

# Realsense installation
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update


# TODO: make this work:
#RUN apt-get update && apt-get install -y \
#    librealsense2-dkms \
#    librealsense2-utils 

RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# Packages for detection and segmentation
RUN git clone https://github.com/dsmircic/masters.git

# Pointcloud to laserscan package
RUN git clone --branch humble https://github.com/ros-perception/pointcloud_to_laserscan.git

# Go to workspace and build it
WORKDIR /root/scout_ws/

# Install dependencies with rosdep
RUN apt-get update 
RUN rosdep update 
RUN rosdep install --from-paths src --ignore-src  --skip-keys=librealsense2 -r -y

RUN apt-get -y upgrade

# Build the workspace
#RUN . /opt/ros/humble/setup.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"
#RUN . /opt/ros/humble/setup.bash
#RUN colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/scout_ws/install/setup.bash" >> ~/.bashrc
WORKDIR /root/scout_ws/
#RUN --mount=type=ssh git clone --branch master git@github.com:mkozlik/scout_startup.git

#WORKDIR /root/scout_ws/scout_startup/
#RUN chmod +x start.sh
#WORKDIR /root/scout_ws/

RUN bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
# Run the container as an interactive shell
CMD ["bash"]
