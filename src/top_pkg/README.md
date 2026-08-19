# top_pkg

Top-level ROS2 bringup package for Livox + RealSense.

## Config

Main bringup switches are in `config/bringup.yaml`:

- `enable_lidar`
- `enable_camera`

## Launch

Bring up selected drivers:

```bash
ros2 launch top_pkg bringup.launch.py
```

Get one synchronized Livox PCD and one RealSense PNG:

```bash
ros2 launch top_pkg one_shot_sync_capture.launch.py
```

Run `bringup.launch.py` first. `one_shot_sync_capture.launch.py` only subscribes to existing topics and saves one synchronized PCD + PNG pair.
The two files use the same five-digit sequence number (for example, `00001.pcd` and
`00001.png`). Each new capture uses one more than the largest existing numeric file
name in either output directory.

The image is converted to `BGR8` before writing PNG, matching the standalone PNG
recorder. The PCD uses the same binary format and 0.5-second LiDAR accumulation
window as `livox_pcd_recorder`'s one-shot capture. `sync_capture.yaml` exposes
the corresponding `lidar_*` and `image_*` settings; LiDAR input supports both
`pointcloud2` and `custom` message types.

Set `max_point_distance_m` in `sync_capture.yaml` to discard points farther than
that many metres from the LiDAR origin while accumulating the synchronized PCD.
Its default value, `0.0`, disables distance filtering.
