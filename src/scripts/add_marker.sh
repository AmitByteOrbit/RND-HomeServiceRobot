#!/bin/sh
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/amit/workspace/RND-HomeServiceRobot/src/worlds/my_new.world " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/amit/workspace/RND-HomeServiceRobot/src/map/map.yaml " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " & 
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black -e " rosrun add_markers add_markers_orig "
