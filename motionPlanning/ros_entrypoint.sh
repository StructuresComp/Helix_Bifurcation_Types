#!/bin/bash
set -e

# Source ROS setup.bash
source /opt/ros/melodic/setup.bash

# Set up X11 forwarding environment variables
export DISPLAY=$DISPLAY
export XAUTHORITY=$XAUTHORITY
export QT_X11_NO_MITSHM=1

# Navigate to the catkin workspace
cd /root/motionPlanning

# install all required packages
source /opt/ros/melodic/setup.bash

apt-get update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make

# Source the setup.bash from the catkin workspace
source /root/motionPlanning/devel/setup.bash

# Start tmux with two panes
tmux new-session -d -s mysession 'roslaunch helix_test helix_setup.launch'
tmux split-window -h 'bash -c "sleep 5 && roslaunch helix_test helix_run.launch; exec bash"'

tmux attach-session -t mysession

# Keep the container running
exec "$@"

