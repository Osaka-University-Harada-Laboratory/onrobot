# OnRobot vacuum gripper

This package contains the URDF files describing OnRobot vacuum grippers (VG10 and VGC10).

```
roslaunch onrobot_vg_description disp_vg10_model.launch  
roslaunch onrobot_vg_description disp_vgc10_1cup_model.launch  
roslaunch onrobot_vg_description disp_vgc10_4cups_model.launch  
```

## Visual and Collision models
### VG10
<img src="images/vg10_visual.png" height="200">  <img src="images/vg10_collision.png" height="200">  

### VGC10 (1 suction cup)
<img src="images/vgc10_visual_1cup.png" height="200">  <img src="images/vgc10_collision_1cup.png" height="200">  

### VGC10 (4 suction cups)
<img src="images/vgc10_visual_4cups.png" height="200">  <img src="images/vgc10_collision_4cups.png" height="200">  

## Reference
- To generate a collision model, you can use [rosmodelgen](https://github.com/takuya-ki/rosmodelgen)
