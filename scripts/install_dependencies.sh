#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo "This dependency list targets Ubuntu 24.04 and ROS 2 Jazzy." >&2
  exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Install ROS 2 Jazzy first; see README.md." >&2
  exit 1
fi
ucon_sudo=()
if [[ "$EUID" -ne 0 ]]; then ucon_sudo=(sudo); fi
"${ucon_sudo[@]}" apt-get update
"${ucon_sudo[@]}" apt-get install -y \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  python3-setuptools python3-numpy python3-scipy \
  libeigen3-dev libpcl-dev libopencv-dev \
  ros-jazzy-ros-base ros-jazzy-pcl-conversions ros-jazzy-cv-bridge \
  ros-jazzy-rviz2 ros-jazzy-tf2-ros
