#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ ! -f install_ucon1/local_setup.bash ]]; then
  echo "Run ./build.sh before testing." >&2
  exit 1
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source install_ucon1/local_setup.bash
export ROS_DOMAIN_ID="${UCON_TEST_DOMAIN:-181}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY
export ROS_LOG_DIR="$PWD/log/tests"
export PYTHONUNBUFFERED=1
for test_file in tests/test_*.py; do
  echo "Running $test_file (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  timeout 120s /usr/bin/python3 "$test_file"
done
