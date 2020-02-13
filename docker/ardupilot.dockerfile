FROM ubuntu:16.04
WORKDIR /ardupilot

RUN useradd -U -d /ardupilot ardupilot && \
    usermod -G users ardupilot

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    python-software-properties && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV USER=ardupilot
ENV DOCKER=y

RUN git clone https://github.com/ArduPilot/ardupilot .
RUN git checkout ArduPlane-stable

COPY docker/ardupilot-fix-docker.patch /ardupilot

RUN git apply ardupilot-fix-docker.patch

RUN chown -R ardupilot:ardupilot /ardupilot && \
    bash -c "Tools/environment_install/install-prereqs-ubuntu.sh -y && apt-get install gcc-arm-none-eabi -y" && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER ardupilot
ENV CCACHE_MAXSIZE=1G
ENV PATH /usr/lib/ccache:/ardupilot/Tools/autotest:/ardupilot/.local/bin:${PATH}

# This should be dev mode only
RUN ./waf configure --board=sitl
RUN ./waf plane

# Open ardupilot master port
EXPOSE 5760

# Pull down wait-for-it.sh, we use this to stop ardupilot from running until it can communicate with gazebo
RUN git clone https://github.com/vishnubob/wait-for-it /ardupilot/wait
