#!/bin/sh
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/amit/workspace/RND-HomeServiceRobot/src/worlds/my.world " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_teleop keyboard_teleop.launch " 
