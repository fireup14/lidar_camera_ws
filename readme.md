# 采集流程说明

## 1. 只开相机，抓单张 PNG
```yaml
enable_lidar: false
enable_camera: true
```
启动相机驱动：
```bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```
```bash
source install/setup.bash
ros2 launch realsense_png_recorder one_shot_png.launch.py
```


## 2. 只开雷达，抓单个 PCD
```yaml
enable_lidar: true
enable_camera: false
```

启动雷达驱动：

```bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```
```bash
source install/setup.bash
ros2 launch livox_pcd_recorder one_shot_pcd.launch.py
```

## 3. 同步抓取 PCD + PNG
```yaml
enable_lidar: true
enable_camera: true
```
先启动两个驱动：
```bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```
```bash
source install/setup.bash
ros2 launch top_pkg one_shot_sync_capture.launch.py
```
```bash
pcl_viewer docs/livox_pcd/00001.pcd
```


