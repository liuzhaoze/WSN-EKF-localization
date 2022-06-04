# WSN-EKF-localization

> WSN Range-Only localization and SLAM with EKF on ROS  
> 基于ROS的Range-Only无线传感器网络扩展卡尔曼滤波定位及SLAM

## Run

```bash
# generate a world
roslaunch uwb_wsn_simulation uwb_wsn_world.launch

# generate internode distance
roslaunch uwb_wsn_simulation uwb_wsn_distance.launch

# run MLE localization server
rosrun uwb_wsn_localization maximum_likelihood_estimation.py

# run EKF SLAM
roslaunch uwb_wsn_localization uwb_wsn_localization.launch
```

## Note

You can use `uwb_wsn_simulation/launch/uwb_sersor_set_generator.py` to generate random sensor nodes.
