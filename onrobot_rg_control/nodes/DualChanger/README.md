# onrobot


ROS nodes for DUAL Quicker Changer  <br />
RG2/RG6 OnRobot Grippers 


## Usage

1. Connect the cable between Compute Box and Dual Tool Changer
2. Connect an ethernet cable between Compute Box and your computer
3. Execute programs

### RG2 / RG6
#### Default configuration     <br />
RG2 - Primary Side   (1)  <br />
RG6 - Secondary Side (2)

#### Send motion commands


##### ROS service call
```
roslaunch onrobot_rg_control bringup_dual.launch ip:=XXX.XXX.XXX.XXX gripper_primary:=rg2 gripper_secondary:=rg6

rosrun onrobot_rg_control OnRobotRGDualServer.py
```
 ##### Control primary Gripper
```
rosservice call /onrobot_rg/set_command_A 'c'
rosservice call /onrobot_rg/set_command_A 'o'
rosservice call /onrobot_rg/set_command_A '!!str 300'
```
##### Control secondary Gripper
```
rosservice call /onrobot_rg/set_command_B 'c'
rosservice call /onrobot_rg/set_command_B 'o'
rosservice call /onrobot_rg/set_command_B '!!str 300'
```

## Author / Contributor



## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
