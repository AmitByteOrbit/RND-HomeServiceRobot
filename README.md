# RND-HomeServiceRobot
## Udacity Robotics Nano Degree - Home Service Robot


This project has a few parts to it. All the parts incrementally build up to the full Home Service Robot. A great project brief to gain a broad understanding of a variety of concepts outline towards the end of this readme.


Packages used and listed in the `src` directory of this repo: <br>

1. **turtlebot** : https://github.com/turtlebot/turtlebot.git : basic drivers for running and using a TurtleBot with ROS.
2. **turtlebot_interactions** : https://github.com/turtlebot/turtlebot_interactions.git : This is the evolution of the turtlebot_viz stack supporting user side interactions with the turtlebot.
3. **turtlebot_simulator** : https://github.com/turtlebot/turtlebot_simulator.git : Launchers for Gazebo simulation of the TurtleBot
4. **slam_gmapping** : https://github.com/ros-perception/slam_gmapping.git : ROS wrapper for OpenSlam's Gmapping.
5. **scripts** : All the shell scripts required for this project.
6. **map** : Map and YAML file for AMCL.
7. **rvizConfig** : Location for RViz config file.
8. **worlds** : The world file used for the project.
9. **pick_objects** : Package created for the robot to navigate to pick-up and drop-off points to pick-up and drop-off markers.
10 **add_markers** : Package created to show and hide markers in RViz.

</br></br>
The shell scripts for each component of the project can be found in the `scripts` directory:
1. **launch.sh** : A test launch script from the first part of the assignment.
2. **test_slam.sh** : Getting the Slam components up and running.
3. **test_navigation.sh** : Testing the AMCL navigation component.
4. **pick_objects.sh** : Launches the `pick_objects` node which I created for this project.
5. **add_marker.sh** : Launches the `add_markers` node which I created for this project. (Actually runs `add_markers_orig` node because the `add_markers` node is modified for the last part.)
6. **home_service.sh** : This script launches the `pick_objects` and the `add_markers` node for the Home Service Robot.

</br></br>
## Simulation Setup
Here is an image of the Turtlebot loaded in my world.

<p align="center"><img src="/misc/robot_gazebo.png" width="800"></p>

## MAPPING
In order to implement SLAM in this project we used a ROS wrapper for Openslam's Gmappaing. This package is called slam_gmapping and is referenced above. This package is capable of creating a 2D occupancy grid map from laser and pose data collected from the mobile robot.

Here is a very partial output from the gmapping test section of the project:

<p align="center"><img src="/misc/slam_test.png" width="800"></p>


## 


