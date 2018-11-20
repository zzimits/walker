[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# walker
Controls turtlebot_gazebo simulation. Drives the robot forward until it gets to close to an object then it stops and turns until it is clear of the object.

## Setup and Build

This package is designed for ROS kinetic. It will be assumed that ROS kinetic is already installed and that a catkin workspace called catkin_ws is already setup on your machine. 

Begin by cloning the directory into the src directory of your catkin workspace.

```
cd /<path to catkin_ws>/src
git clone https://github.com/zzimits/walker
cd ~/catkin_ws
. ~/catkin_ws/devel/setup.bash
catkin_make
```
## Running

In one terminal ensure that roscore is running by executing:

````
roscore
````
If the gazebo is already running and you would like to start the walker node run the following command:
```
rosrun walker controller
```

To launch all the necessary files to the turtlebot_gazebo simulation in turtlebot_world use the following command: 
```
roslaunch walker walker.launch
```

## Bag Files
To start a bag file use the launch file with the following input argument
```
roslaunch walker walker.launch record_chatter:=true
```
To inspect the bag file navigate to the results directory and use the rosbag info command
```
cd /<path to directory>/results
rosbag info bagfile.bag
```
To play back a bag file
```
cd /<path to directory>/results
rosbag play bagfile.bag
```