# time_rewrite_node

Rewrites timestamps of selected PointCloud2 topics and TF to the node's current clock time.

## Interfaces
- Subscribes:
  - `/control/carla_ros_bridge/points/lidar_left` (`sensor_msgs/msg/PointCloud2`)
  - `/control/carla_ros_bridge/points/lidar_rear` (`sensor_msgs/msg/PointCloud2`)
  - `/control/carla_ros_bridge/points/lidar_right` (`sensor_msgs/msg/PointCloud2`)
  - `/tf_old` (`tf2_msgs/msg/TFMessage`)
- Publishes:
  - `/control/carla_ros_bridge/points/lidar_left_fixed` (`sensor_msgs/msg/PointCloud2`)
  - `/control/carla_ros_bridge/points/lidar_rear_fixed` (`sensor_msgs/msg/PointCloud2`)
  - `/control/carla_ros_bridge/points/lidar_right_fixed` (`sensor_msgs/msg/PointCloud2`)
  - `/tf` (`tf2_msgs/msg/TFMessage`)

## Run
```bash
ros2 run ros2_utils time_rewrite_node
```
