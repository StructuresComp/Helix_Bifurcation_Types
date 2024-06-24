# Use the official ROS Melodic image as a base
FROM ros:melodic

# Install additional packages (if needed)
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    xterm \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Set up entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
# Copy your local ROS workspace into the container

RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]


CMD ["bash"]

