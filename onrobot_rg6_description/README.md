# OnRobot RG6 two-fingered gripper

This package contains the URDF files describing OnRobot RG6.

## Display the gripper URDF description
```
$ roslaunch onrobot_rg6_description disp_rg6_model.launch 
```

### Visual and Collision models
<img src="images/visual.png" height="200"> <img src="images/collision.png" height="200">  

## Gazebo simulation
```
$ roslaunch onrobot_rg6_description bringup_rg6_gazebo.launch
$ rostopic pub -1 /onrobot_rg6/joint_position_controller/command std_msgs/Float64 "data: 0.5"
```

## Reference
- To generate a collision model, you can use [rosmodelgen](https://github.com/takuya-ki/rosmodelgen)
