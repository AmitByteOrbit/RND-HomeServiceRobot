#!/bin/sh
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black  -e  " gazebo " &
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm -fa "DejaVu Sans Mono" -fs 12 -fg white -bg black  -e  " rosrun rviz rviz"
