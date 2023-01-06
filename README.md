# Postrobot - Turtlebot3
In this repository you can find the code of the project of the subject "Mobile Robots" of the Robotics Degree of the University of Alicante.

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
