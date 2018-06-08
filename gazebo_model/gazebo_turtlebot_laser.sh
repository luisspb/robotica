#!/bin/bash
source /opt/ros/indigo/setup.bash
export ROS_IP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')
roslaunch turtlebot_gazebo turtlebot_laser.launch
