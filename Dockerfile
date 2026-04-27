FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    ROS_DISTRO=humble \
    COLCON_WS=/workspace \
    CMAKE_VERSION=3.31.3

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    wget \
    ninja-build \
    clang \
    clang-format \
    clang-tidy \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    locales \
    gosu \
    sudo \
    procps \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgpiod-dev \
    libopencv-dev \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-gscam \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh \
    && chmod +x cmake-${CMAKE_VERSION}-linux-x86_64.sh \
    && ./cmake-${CMAKE_VERSION}-linux-x86_64.sh --skip-license --prefix=/usr/local \
    && rm cmake-${CMAKE_VERSION}-linux-x86_64.sh

RUN rosdep init || true && rosdep update

# Workspace skeleton
RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/bash.bashrc && \
    echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /etc/bash.bashrc

COPY docker_entrypoint.sh /ros_entry.sh
RUN chmod +x /ros_entry.sh

ENTRYPOINT ["/ros_entry.sh"]
CMD ["bash"]

