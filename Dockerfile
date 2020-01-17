# Custom ROS Base image for SUAS
# Based off of official base image for ROS Melodic
# Add dependencies like rosserial and gazebo as appropriate

FROM ros:melodic-ros-base-bionic AS base

ENV USERNAME=docker
ENV USER_UID=1000
ENV USER_GID=$USER_UID

ENV ROS_DISTRO melodic
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get install -q -y ros-melodic-rosserial \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash/ --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "source /opt/ros/melodic/setup.sh" > /home/${USERNAME}/.bashrc \
    && apt-get autoremove && apt-get clean \
    && mkdir /home/${USERNAME}/catkin_ws

FROM base as production

WORKDIR /catkin_ws/src

COPY src .
COPY entrypoint.sh .
RUN touch ./suas_sim/CATKIN_IGNORE \
    && /bin/bash -c 'source /opt/ros/melodic/setup.sh && cd /catkin_ws && catkin_make' \
    && chown -R $USERNAME:$USER_GID /catkin_ws \
    && echo "source /catkin_ws/devel/setup.sh" > /home/${USERNAME}/.bashrc \
    && chmod 0755 entrypoint.sh

ENV DEBIAN_FRONTEND dialog
USER $USERNAME

ENTRYPOINT ["/catkin_ws/src/entrypoint.sh"]
