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

# Add gazebo user
RUN useradd -U -d /home/gazebo gazebo && \
    usermod -G users gazebo

# Create home dir
RUN mkdir -p /home/gazebo

# Give ownership to gazebo user
RUN chown -R gazebo:gazebo /home/gazebo

USER gazebo

# Move to home dir
WORKDIR /home/gazebo

# Pull down ardupilot-gazebo plugin
RUN git clone https://github.com/SwiftGust/ardupilot_gazebo ./ardupilot-gazebo

# Move into ardupilot-gazebo plugin sources
WORKDIR /home/gazebo/ardupilot-gazebo

# Checkout gazebo 9 tag
RUN git checkout gazebo9

# Remove the .hg files in the models because gazebo thinks its a model and looks for a model.config (which doesn't exist obviously)
RUN rm -rf ./gazebo_models/.hg

# Copy over the setup.sh (meant to be sourced)
COPY ./gazebo-setup.sh /home/gazebo/setup.sh

RUN echo source /home/gazebo/setup.sh >> /home/gazebo/.bashrc

# Open FDM ports
EXPOSE 9002/udp
EXPOSE 9003/udp

RUN bash -c 'mkdir build && \
        cd build && \
        cmake .. && \
        make -j4' # \
        # gazebo --verbose ./gazebo_worlds/zephyr_ardupilot_demo.world
