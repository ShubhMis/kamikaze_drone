#!/usr/bin/env bash
# Clean environment for this child process; does not change the user's shell.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH=/usr/local/bin:/usr/bin:/bin
export PYTHONNOUSERSITE=1
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
source /opt/ros/humble/setup.bash
source "$ROOT/drone_ws/install/setup.bash"
exec /usr/bin/python3 "$ROOT/drone_ws/src/drone_follow/scripts/baseline.py" "$@"
