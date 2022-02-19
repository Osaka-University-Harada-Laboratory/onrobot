# onrobot

ROS drivers for OnRobot Grippers

# Features

- ROS Noetic (Python3)
- OnRobot RG2/RG6 controler via Modbus TCP
- OnRobot VG10/VGC10 controler via Modbus TCP

# Installation

	$ git clone git@github.com:takuya-ki/onrobot.git catkin_ws/src; cd catkin_ws
    $ sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y
	$ catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

# Usage

### RG2/RG6
    $ roslaunch onrobot_rg_control bringup.launch 
    $ rosrun onrobot_rg_control OnRobotRGSimpleController.py

### VG10/VGC10
    $ roslaunch onrobot_vg_control bringup.launch 
    $ rosrun onrobot_vg_control OnRobotVGSimpleController.py  

# Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)
