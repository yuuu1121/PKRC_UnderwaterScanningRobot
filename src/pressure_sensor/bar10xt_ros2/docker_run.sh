#!/usr/bin/env bash
# Build (if needed) and run the Bar10XT ROS2 Humble node in Docker.
# Usage: ./docker_run.sh [launch args...]
#   e.g. ./docker_run.sh pressure_offset:=0.007 water_density:=1025.0
set -e

IMAGE=bar10xt:humble
I2C_DEV=/dev/i2c-1
DIR="$(cd "$(dirname "$0")" && pwd)"

# Build the image if it does not exist yet
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo ">> building $IMAGE ..."
    docker build -t "$IMAGE" "$DIR"
fi

echo ">> running $IMAGE (i2c: $I2C_DEV)"
exec docker run -it --rm \
    --name bar10xt \
    --network host \
    --ipc host \
    --device "$I2C_DEV" \
    "$IMAGE" \
    bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bar10xt_ros2 bar10xt.launch.py $*"
