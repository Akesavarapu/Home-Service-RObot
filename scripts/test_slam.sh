#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 10
xterm -e "  roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 14
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
