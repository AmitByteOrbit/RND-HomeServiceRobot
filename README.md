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

<p align="center"><img src="/misc/robot_gazebo2.png" width="800"></p>

## MAPPING
Simultaneous localization and mapping, or SLAM for short, is the process of creating a map using a robot or unmanned vehicle that navigates that environment while using the map it generates.
In order to implement SLAM in this project we used a ROS wrapper for Openslam's Gmappaing. This package is called slam_gmapping and is referenced above. This package is capable of creating a 2D occupancy grid map from laser and pose data collected from the mobile robot.

Here is a very partial output from the gmapping test section of the project:

<p align="center"><img src="/misc/slam_test.png" width="800"></p>

</br></br>
## PGM Map File
The map ROS AMCL Package uses is a pgm file. A pgm file is a grayscale image file. As outlined in the Localization project I used the PGM_Map_Creator (https://github.com/hyfan1116/pgm_map_creator) to create a PGM map file. The file is located in the /src/map foler along with the associated YAML file.

Here is what my PGM file looks like:
<p align="center"><img src="/misc/map.png" width="800"></p>

</br></br>
## Localization and Navigation
For localization and navigation we used the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm. The Uniform-cost search is a searching algorithm used for traversing a weighted tree or graph. The ROS navigation stack helps the robot to avoid any obstacle on its path by re-planning a new trajectory once your robot encounters them. The localization aspect is achieve using AMCL (adaptive monte carlo localization) which is an algorithm for robots to localize using a particle filter. 
The ROS navigation stack is a 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

Here is the robot busy with Localization and Navigation using rviz 2d nav goals.

<p align="center"><img src="/misc/nav1.png" width="600"></p>

</br></br>
## Pick Objects
The `pick_objects` node involved sending simple goals to the the Navigation Stack so that the robot to navigate to the goals. For me the main aspect was to create a goal and send it to move_base using the move_base_msgs::MoveBaseGoal message type which is included automatically with the MoveBaseAction.h header. This node can be tested using the `pick_objects.sh` script.

</br></br>
## Add Markers
There were two version of this node that I created. One is called `add_markers_orig` which was the add markers test and functions with the `add_markers.sh` script. This node functioned as both - a publisher and a subscriber.</br></br>
The publisher published the marker under the `visualization_marker` topic. This was displayed in RViz using the *Marker* view.
</br>`ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);`</br></br>
The subscriber subscribed to the robot's odometry on the `odom` topic. This was used to calculate the robots pose and determine if the robot was at a pick-up or drop-off zone.
</br>`ros::Subscriber robot_sub = n.subscribe("/odom",1000,robotPoseCallback);`

</br></br>
## Home service in action

Here is a little video clip of the Home Service Robot in action once everything was completed.
<p align="center"><img src="/misc/home-service-robot-vid.gif" width="800"></p>
