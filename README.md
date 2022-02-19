# onrobot

ROS drivers for OnRobot Grippers

# Features

- ROS Noetic (Python3)
- OnRobot RG controler via Modbus TCP

# Installation

	$ git clone git@github.com:takuya-ki/onrobot.git catkin_ws/src; cd catkin_ws
    $ sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y
	$ catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

# Usage

    $ roscore
    $ rosrun onrobot_rg_control OnRobotRGStatusListener.py
    $ rosrun onrobot_rg_control OnRobotRGTcpNode.py
    $ rosrun onrobot_rg_control OnRobotRGSimpleController.py

# Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)
