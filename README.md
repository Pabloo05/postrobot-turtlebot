# Postrobot - Turtlebot3
In this repository you can find the code of the project of the subject "Mobile Robots" of the Robotics Degree of the University of Alicante.

# Description
This project consists of a robot that has to move around a map and deliver a package to a specific point. The robot has to avoid obstacles and follow the path to the destination. In the meantime, the robot patrols the map going from one point to another.

# Demo
https://user-images.githubusercontent.com/63607884/212547884-883b64c8-36ec-46ef-9df1-d304bfb8fc16.mp4

# Requirements
- Turtlebot3 Waffle (`export TURTLEBOT3_MODEL=waffle`)
  
# Installation

In order to install this package, you need run the following commands:
```bash
git clone https://github.com/Pabloo05/postrobot-turtlebot.git
cd postrobot-turtlebot
catkin_make
source devel/setup.bash
```

# Usage

In order to use this package, you need to run the following commands:
```bash
roslaunch core turtlebot3_empty_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
rosrun map_server map_server map.yaml
python3 src/core/scripts/main.py
```
## Delivery mode
To use the delivery mode, you must post a message in the "delivery" topic with the name of the destination point. For example
```bash
rostopic pub /delivery std_msgs/String "data: 'o1'"
```
With this command, the robot will go to point "o1" and deliver the package.
The points are defined in offices list in the main.py file.

## Patrol mode
Patrol mode is activated while the robot is not delivering any packages. The robot will go from one point to another to patrol the map. The points are defined in waypoints list in the main.py file.

## Camera
The camera is activated by default when the script is run. It is used for monitoring the robot and for surveillance in patrol mode.

# Authors
- Pablo García Gómez
