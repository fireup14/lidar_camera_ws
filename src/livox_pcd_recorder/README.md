# livox_pcd_recorder

ROS2 package for recording Livox `sensor_msgs/msg/PointCloud2` data and exporting periodic `.pcd` files.
It also supports `livox_ros_driver2/msg/CustomMsg`.

## Parameters

- `input_topic`: subscribed point cloud topic, default `/livox/lidar`
- `message_type`: `pointcloud2` or `custom`
- `output_dir`: directory for exported PCD files, default `docs/livox_pcd`.
  Relative paths are resolved from the directory where `ros2 launch` is run.
- `save_duration_sec`: accumulation time for each exported PCD
- `save_binary`: save binary PCD if `true`, otherwise ASCII
- `save_on_shutdown`: flush remaining buffered points when the node exits
- `save_once`: save one PCD and exit after the first flush

PCD files are named `00001.pcd`, `00002.pcd`, and so on. Before every save,
the recorder scans `output_dir` for PCD files whose basename is numeric and
uses the number after the largest existing one, so historical captures are not
overwritten.

## Calibration defaults

- Default mode is tuned for camera-lidar calibration capture
- `message_type: pointcloud2`
- `save_duration_sec: 0.5`

Use the combined launch below to start the Livox driver in `PointCloud2` mode together with the recorder.

## Livox driver match

- If `livox_ros_driver2` uses `xfer_format: 0`, set `message_type: pointcloud2`
- If `livox_ros_driver2` uses `xfer_format: 1`, set `message_type: custom`

## Run

```bash
colcon build --packages-select livox_pcd_recorder
source install/setup.bash
ros2 launch livox_pcd_recorder pcd_recorder.launch.py
```

Save one PCD and exit:

```bash
ros2 launch livox_pcd_recorder one_shot_pcd.launch.py
```

Or start driver + recorder together:

```bash
ros2 launch livox_pcd_recorder calibration_capture.launch.py
```
