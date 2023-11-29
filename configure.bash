#!/bin/bash

# Setup inicial
source /opt/ros/humble/setup.bash
# cd ~/turtlebot3_ws && colcon build --symlink-install
source ~/turtlebot3_ws/install/setup.bash

# Necessarios para rodar o gazebo
export DISPLAY=:0
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3
