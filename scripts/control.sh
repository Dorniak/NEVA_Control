#!/bin/bash
source /opt/ros/foxy/setup.bash
source /home/nevabn/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
screen -S control -d -m ros2 launch NEVA_control control.launch.py