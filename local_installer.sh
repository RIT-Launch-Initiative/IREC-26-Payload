#!/usr/bin/env bash
set -euo pipefail

CMAKE_VERSION=3.31.3
ROS_DISTRO=humble
COLCON_WS=/workspace

export DEBIAN_FRONTEND=noninteractive
apt_update() {
  sudo apt-get update -y
}

# Repos and base prep
apt_update
sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo mkdir -p /usr/share/keyrings
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg /etc/apt/keyrings/ros-archive-keyring.gpg /etc/apt/sources.list.d/ros2.list
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
apt_update
sudo apt-get upgrade -y
sudo apt-get install -y --no-install-recommends \
  locales sudo gosu procps \
  build-essential git wget ninja-build clang clang-format clang-tidy \
  python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool python3-argcomplete \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgpiod-dev libopencv-dev \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-vision-opencv \
  ros-${ROS_DISTRO}-gscam
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

# CMake 3.31.3
# Check aarch64 or x86_64
ARCH=$(uname -m)
if [ "$ARCH" != "x86_64" ] && [ "$ARCH" !=
    "aarch64" ]; then
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

cd /tmp
CMAKE_TAR="cmake-${CMAKE_VERSION}-Linux-${ARCH}.tar.gz"
wget "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_TAR}"
tar -xzvf "${CMAKE_TAR}"
sudo cp -r "cmake-${CMAKE_VERSION}-linux-${ARCH}/"* /usr
cd -
rm -rf /tmp/cmake-*

# rosdep setup
sudo rosdep init || true
rosdep update