# DualChanger

## Features

- ROS nodes for DUAL Quicker Changer  
- RG2/RG6 OnRobot Grippers 

## Usage

1. Connect the cable between Compute Box and Dual Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs

### RG2 / RG6
#### Default configuration  
- RG2 - Primary Side   (1)  
- RG6 - Secondary Side (2)  
*Please check the web client. In our case, the side with the socket of the connection cable was primary.

#### Send motion commands
##### ROS service call
```
roslaunch onrobot_rg_control bringup_dual.launch ip:=XXX.XXX.XXX.XXX gripper_primary:=rg2 gripper_secondary:=rg6
rosrun onrobot_rg_control OnRobotRGDualServer.py
rosservice call /onrobot_rg/set_command_A 'c'
rosservice call /onrobot_rg/set_command_A 'o'
rosservice call /onrobot_rg/set_command_A '!!str 300'
rosservice call /onrobot_rg/set_command_B 'c'
rosservice call /onrobot_rg/set_command_B 'o'
rosservice call /onrobot_rg/set_command_B '!!str 300'
rosservice call /onrobot_rg/restart_power
```
