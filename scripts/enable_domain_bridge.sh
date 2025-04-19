#!/bin/bash

set -euo pipefail

docker exec -d moveit2_container_left \
    bash -c "source ./install/setup.bash && ros2 run domain_bridge domain_bridge /workspaces/src/robot_piano/config.yaml"

docker exec -d moveit2_container_left \
    bash -c "source ./install/setup.bash && ros2 run robot_piano bridge"
