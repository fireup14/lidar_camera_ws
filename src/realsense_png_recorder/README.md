# realsense_png_recorder

ROS2 package for subscribing to a RealSense RGB image topic and saving PNG images.
Each run saves a single PNG image and then exits.

## Parameters

- `input_topic`: RGB image topic, default `/camera/camera/color/image_raw`
- `output_dir`: PNG output directory, default `/home/fire/Desktop/Lidar_Camera_Calibrator/lidar_camera_ws/docs/realsense_png`

PNG files are named `00001.png`, `00002.png`, and so on. Before each save, the
recorder scans `output_dir` for PNG files whose basename is numeric and uses
the next number after the largest existing one, so existing captures are not
overwritten.

## Run

Start the camera publisher first, in a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py enable_rviz:=false
```

Confirm that it is publishing frames; this command must show a non-zero rate:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

Then start the one-shot recorder in another terminal:

```bash
colcon build --packages-select realsense_png_recorder
source install/setup.bash
ros2 launch realsense_png_recorder one_shot_png.launch.py
```
