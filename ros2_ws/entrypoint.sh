#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /workspace/coop-swarm/ros2_ws/install/setup.bash
cd /workspace/coop-swarm
exec "$@"
