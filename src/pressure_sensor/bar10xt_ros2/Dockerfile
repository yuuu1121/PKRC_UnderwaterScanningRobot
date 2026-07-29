# ROS2 Humble for the Keller Bar10XT sensor — runs on a JetPack 5 / Ubuntu 20.04
# Jetson via Docker (host has no native Humble). arm64 base image.
FROM ros:humble-ros-base

# I2C access from Python (smbus2) + colcon build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        i2c-tools \
    && pip3 install --no-cache-dir smbus2 \
    && rm -rf /var/lib/apt/lists/*

# Copy the package into a colcon workspace and build it
WORKDIR /ros2_ws
COPY . /ros2_ws/src/bar10xt_ros2
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-select bar10xt_ros2

# Source the workspace on every shell / run
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

# Default: launch the sensor node
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch bar10xt_ros2 bar10xt.launch.py"]
