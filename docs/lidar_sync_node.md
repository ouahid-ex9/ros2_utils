# lidar_sync_node

Synchronizes multiple PointCloud2 topics to the latest message with timestamp <= `/clock + offset`, republishing to `<input>/synced`.

## Parameters
- `offset` (double, seconds): Time offset relative to `/clock` for message selection. Default: `0.0`.
- `lidar_topics` (string list): Input topics to synchronize.

Example config:
```yaml
lidar_sync_node:
  ros__parameters:
    offset: 0.0
    lidar_topics:
      - "/front/points"
      - "/rear/points"
```

## Interfaces
- Subscribes:
  - `/clock` (`rosgraph_msgs/msg/Clock`)
  - Each topic in `lidar_topics` (`sensor_msgs/msg/PointCloud2`)
- Publishes:
  - `<input>/synced` for each input (`sensor_msgs/msg/PointCloud2`)

## Run
```bash
ros2 run ros2_utils lidar_sync_node --ros-args --params-file $(ros2 pkg prefix ros2_utils)/share/ros2_utils/config/config.yaml
```
