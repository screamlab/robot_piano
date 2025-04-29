#!/bin/bash
set -euo pipefail

docker exec -t moveit2_container_left \
    bash -c "source ./install/setup.bash && ros2 run robot_piano left" &

docker exec -t moveit2_container_right \
    bash -c "source ./install/setup.bash && ros2 run robot_piano right"
