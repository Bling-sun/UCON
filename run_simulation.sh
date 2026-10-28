#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -f install_ucon1/setup.bash ]]; then
  echo "Run ./build.sh first."
  exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source install_ucon1/local_setup.bash
export ROS_LOG_DIR="$PWD/log/runtime"
exec ros2 launch fapp_planner sim_planner.launch.py "$@"
