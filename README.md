# onrobot

ROS drivers for OnRobot Grippers

## Features

- ROS Noetic (Python3)
- Controller for OnRobot RG2 / RG6 via Modbus/TCP
- Controller for OnRobot VG10 / VGC10 via Modbus/TCP

## Installation

	$ git clone git@github.com:takuya-ki/onrobot.git catkin_ws/src; cd catkin_ws
	$ sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y --os=ubuntu:focal -y
	$ catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3

## Usage

1. Connect the cable between Compute Box and Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs

### RG2 / RG6

##### Send motion commands
    $ roslaunch onrobot_rg_control bringup.launch gripper:=[rg2/rg6]
    $ rosrun onrobot_rg_control OnRobotRGSimpleController.py

##### Visualize a model
    $ roslaunch onrobot_rg6_visualization disp_onrobot_rg6_model.launch
    $ roslaunch onrobot_rg2_visualization disp_onrobot_rg2_model.launch

### VG10 / VGC10

##### Send motion commands
    $ roslaunch onrobot_vg_control bringup.launch
    $ rosrun onrobot_vg_control OnRobotVGSimpleController.py  

##### Visualize a model
    $ roslaunch onrobot_vgc10_visualization disp_onrobot_vgc10_model.launch
    $ roslaunch onrobot_vg10_visualization disp_onrobot_vg10_model.launch

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
