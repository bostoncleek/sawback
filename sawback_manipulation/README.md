# sawback_manipulation

# Overview
This package contains the grasping node and the pick and place node.

# Nodes:
## grasp_detection
This node uses the [grasp pose detection](https://github.com/atenpas/gpd#install) library to find grasp candidates given a point cloud. It is capable of segmenting the ground plane from the grasped object. It can also remove a portion of the point cloud specified by the user. It subscribes to `sensor_msgs/PointCloud2` on the `cloud` topic. The filtered point cloud is published on `filtered_cloud`. When the `get_grasp` service is called the highest ranked grasp is returned. The highest ranked grasp is represented as a `geometry_msgs/PoseStamped` in the `base` frame of the Sawback.

## sawback_pick_place
This node contains an action server named `pick_place` that can execute pick and place tasks using the custom [pick and place API](https://github.com/bostoncleek/sawback/blob/master/sawback_manipulation/include/sawback_manipulation/pick_place.hpp). When the action server is called with the task name `pick` the node calls the `get_grasp` service to find the best grasp candidate. When the task is set to `place` the user must define the place location in the `base` frame of the Sawback.

# Launch
To launch the full pick/place pipeline using the grasp pose detection library on the Sawback:
```
roslaunch sawback_manipulation sawback_manipulation.launch
```

To launch the manipulation pipeline using fake controllers:
```
roslaunch sawback_manipulation sawback_manipulation.launch fake_execution:=true use_rviz:=true
```

If you are launching from a personal computer you can visualize the results:
```
roslaunch sawback_manipulation visualization.launch
```

To launch only the grasping node:
```
roslaunch sawback_manipulation grasping.launch
```
