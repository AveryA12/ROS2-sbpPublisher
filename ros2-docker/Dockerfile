# Dockerfile
FROM ros:humble-ros-base

# Python and pip
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

 
# Install minimal ROS 2 and demo talker/listener
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-nmea-navsat-driver \
  ros-humble-robot-localization \
  ros-humble-imu-tools \
  ros-humble-imu-filter-madgwick

# Python packages for sbp_publisher
# Need git so pip can pull from GitHub
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
&& rm -rf /var/lib/apt/lists/*
 
# Install Swift Navigation's libsbp (Python package "sbp") and pyserial
RUN pip3 install --no-cache-dir \
    "git+https://github.com/swift-nav/libsbp.git#subdirectory=python" \
    pyserial

# Convenience: always have ROS env loaded in interactive shells
RUN bash -lc "echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc"
 
# Default working dir
WORKDIR /root
