# Rapidly-Exploring Random Tree


A path planning algorithm based on RRT implemented using ROS.

distro - kinetic 

The algorithm find an optimized path for a multi-obstacle environment. The visualtization is done in RVIZ and the code is written in python. 

The package has one executable: 

test.py

RVIZ parameters: 

Frame_id = "path_planning" 
marker_topic = "path_planning/viz" 
Instructions: 

Open terminal and type 
$roscore 
Open new terminal and go to the the root of your catkin workspace 
$catkin build 
$source ./devel/setup.bash 
$roslaunch rrt test.launch 
open new terminal 
$rviz
In the RVIZ window, change: 
fixed frame under global option to "path_planning" 
add a marker and change marker topic to "path_planning/viz" 
The environment and the path will be visualized in RVIZ. 
