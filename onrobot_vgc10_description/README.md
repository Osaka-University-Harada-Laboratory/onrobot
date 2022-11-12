# OnRobot VGC10 two-fingered gripper

This package contains the URDF files describing OnRobot VGC10.

To display the gripper URDF description

## One suction cup type
```
$ roslaunch onrobot_vgc10_description disp_onrobot_vgc10_1cup_model.launch 
```

### Visual model
<img src="images/visual_1cup.png" height="300">  

### Collision model
<img src="images/collision_1cup.png" height="300">  

## Four suction cups type
```
$ roslaunch onrobot_vgc10_description disp_onrobot_vgc10_4cups_model.launch 
```

### Visual model
<img src="images/visual_4cups.png" height="300">  

### Collision model
<img src="images/collision_4cups.png" height="300">  

## Reference
- To generate a collision model, you can use [rosmodelgen](https://github.com/takuya-ki/rosmodelgen)
