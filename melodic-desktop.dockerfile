FROM osrf/ros:melodic-desktop-full-bionic

# Setup sources.list to have ros package repos
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up keys for ROS
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ros packages
RUN apt-get update && apt-get install -y \
    tmux \
    curl \
    vim-gtk3 \
    sudo \
    libgl1-mesa-dri \
    mesa-utils \
    p7zip-full \
    libgl1-mesa-glx \
    python \
    python-pip \
    python3 \
    python3-pip \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Install catkin build dependencies
RUN pip3 install catkin_pkg

# Install GeographicLib datasets (MAVROS needs it for geo related items)
RUN curl https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | /bin/bash
