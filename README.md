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

## 文件解析

### uwb_wsn_world.launch

> 位置：uwb_wsn_simulation/launch

该 launch 文件调用了同目录的以下 4 个 launch 文件：

* `uwb_wsn_scenario.launch`：加载机器人所在场景模型，设置机器人生成位置。
* `uwb_anchor_set.launch`：在场景中加载锚节点，锚节点的本质为 TF(static_transform) 坐标系。
* `uwb_unknown_set.launch`：在场景中加载未知节点，原理同上。
* `navigation.launch`：实现机器人的导航，旨在方面控制机器人移动。

### uwb_wsn_distance.launch

> 位置：uwb_wsn_simulation/launch

该 launch 文件运行了 `uwb_wsn_simulation/scripts` 目录下的 2 个 Python 脚本。

* `uwb_wsn_distance.py`：根据机器人的坐标、节点 TF 的坐标，计算机器人与节点、节点与节点之间的距离，并加入高斯噪声，以模拟节点的测距结果。单个节点的测距信息格式为 `uwb_node_data.msg` ，网络中所有节点的测距信息格式为 `uwb_wsn_data.msg` (path: uwb_wsn_simulation/msg)。可使用命令 `rostopic echo /uwb_wsn_data_topic` 查看所有节点的测距信息。（注：第 29 行设置了节点间的通信范围）
* `plot_communication_state.py`：该脚本实现了节点连通性可视化。

### maximum_likelihood_estimation.py

> 位置：uwb_wsn_localization/scripts

该 Python 脚本实现了迭代的极大似然估计定位法。供 EKF 定位发现新节点时调用。

### uwb_wsn_localization.launch

> 位置：uwb_wsn_localization/launch

该 launch 文件运行了 `uwb_wsn_localization/scripts` 目录下的 2 个 Python 脚本。

* `improved_ekf_localization_with_internode_measurement.py`：实现了使用节点间测距信息的 EKF SLAM 算法。
  * 基本思想：
  * 1、根据锚节点的位置信息和锚节点到未知节点的距离信息对尽可能多的未知节点进行极大似然估计定位（Line 22, `initialization()`）。
  * 2、根据里程计信息对状态向量进行预测（Line 83, `prediction_step()`）。
  * 3、根据 WSN 中的测距信息对状态向量进行修正（Line 130, `correction_step()`）。如果遇到新节点，则调用极大似然估计定位法确定新节点的坐标，并更新到状态向量中。
  * 4、将 EKF SLAM 估计结果的状态向量 `mu` 和协方差矩阵 `sigma` 发布到话题 `ekf_localization_data` ，消息格式为 `uwb_wsn_slam_data.msg` (path: uwb_wsn_localization/msg)。可使用命令 `rostopic echo /ekf_localization_data` 查看估计结果。
* `plot_slam_result.py`：根据 ekf_localization_data 话题中的估计结果实现可视化。
