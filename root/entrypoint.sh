#!/bin/bash
set -e

# setup ros2 environment
source "/opt/colcon_ws/install/setup.bash" --
exec "$@"