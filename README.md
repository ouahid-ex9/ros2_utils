# ros2_utils

ROS 2 utility nodes for time publication, LiDAR synchronization, and timestamp rewriting.

## Package
- Name: `ros2_utils`
- Build system: `ament_cmake`

## Build
From your workspace root:
```bash
colcon build --packages-select ros2_utils
source install/setup.bash
```

## Nodes

### clock_publisher_node
Publishes the ROS `/clock` topic at 100 Hz from the node's clock.
- Publishes:
  - `/clock` (`rosgraph_msgs/msg/Clock`)

Run:
```bash
ros2 run ros2_utils clock_publisher_node
```

---

### lidar_sync_node
Synchronizes multiple `sensor_msgs/msg/PointCloud2` topics to the latest message with timestamp <= `/clock + offset`, and republishes to `<input>/synced`.

- Parameters:
  - `offset` (double, seconds): Time offset applied to `/clock` when selecting messages. Default: `0.0`.
  - `lidar_topics` (string list): Input topics to synchronize. Default in code: `["/lidar1/points", "/lidar2/points"]`.

- Subscribes:
  - `/clock` (`rosgraph_msgs/msg/Clock`)
  - Each topic in `lidar_topics` as `sensor_msgs/msg/PointCloud2`

- Publishes:
  - For each input `<topic>`, publishes to `<topic>/synced` (`sensor_msgs/msg/PointCloud2`)

Example config (`config/config.yaml`):
```yaml
lidar_sync_node:
  ros__parameters:
    offset: 0.0
    lidar_topics:
      - "/front/points"
      - "/rear/points"
```

Run with config:
```bash
ros2 run ros2_utils lidar_sync_node --ros-args --params-file $(ros2 pkg prefix ros2_utils)/share/ros2_utils/config/config.yaml
```

---

### time_rewrite_node
Rewrites timestamps for select `sensor_msgs/msg/PointCloud2` topics and TF messages to the node's current clock time.

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

Run:
```bash
ros2 run ros2_utils time_rewrite_node
```

## Notes
- `/clock` publisher is useful when running with simulated or controlled time.
- `lidar_sync_node` uses best-effort QoS for `/clock` and queue size 10 for PointCloud2 topics.
