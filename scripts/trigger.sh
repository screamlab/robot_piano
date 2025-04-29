#!/bin/bash

# Function to handle SIGINT (Ctrl-C)
cleanup() {
    echo "Received SIGINT, stopping background processes..."
    # Kill all background jobs started by this script
    kill -9 $(jobs -p)
    exit 0
}

# Set up trap for SIGINT
trap cleanup SIGINT

# Run both commands concurrently in the background
# ros2 service call /left_start_sync std_srvs/srv/Trigger &
# ros2 service call /right_start_sync std_srvs/srv/Trigger &
docker exec -it moveit2_container_left bash -c "source ./install/setup.bash && ros2 run robot_piano trigger"

# Wait for both background processes to finish
wait
