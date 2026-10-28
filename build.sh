#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required. See README.md." >&2
  exit 1
fi
# Do not record another workspace as an underlay in the generated setup files.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export MAKEFLAGS="${MAKEFLAGS:--j4}"
colcon --log-base log/ucon1_rebuild build \
  --base-paths src \
  --build-base build_ucon1 \
  --install-base install_ucon1 \
  --symlink-install \
  --parallel-workers "${UCON_BUILD_WORKERS:-2}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
