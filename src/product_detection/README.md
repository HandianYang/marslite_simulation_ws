# Product Detection

## Prerequisites

Hardware
* Realsense D435/D435i camera

Software
* Docker image: `handianyang/marslite-simulation`
* Python 3.10
    ```bash
    $ source src/product_detection/venv-3-10/bin/activate
    ```

## Try examples

### Visualize object detection
```bash
$ roslaunch product_detection object_detection_viewer_example.launch
```

### Visualize point clouds
```bash
$ roslaunch product_detection opencv_pointcloud_viewer_example.launch
```

### Build point clouds with a single Realsense RGB-D camera (no robots)

#### 1. Launch RGB-D sensor(s)

* For Realsense D435
```bash
$ roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
```
* For Realsense D435i
```bash
# ISSUE: Use unite_imu_method:="copy" if imu topics keep stopping
$ roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
    enable_accel:=true

$ rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
```

#### 2. Build point clouds

* For Realsense D435
```bash
$ roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
```
* For Realsense D435i
```bash
 $ roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu
```


### Build point clouds with simulated marslite

#### 1. Launch the marslite simulation environment

```bash
$ roslaunch mars_lite_description gazebo_supermarket.launch realsense_enabled:=true
```

This script publishes required topics for building point clouds such as
* base link of the robot (`/base_link`)
* odometry of the robot (`/odom_filtered`)
* 2D lidar (`/scan`)
* RGB image (`/camera1/color/image_raw`)
* depth image (`/camera1/depth/image_rect_raw`)
* meta information of Realsense cameras (`/camera1/color/camera_info`)


#### 2. Build point clouds with `camera1`
```bash
$ roslaunch product_detection rtabmap_pointcloud.launch
```

The point-cloud result will be stored at `/home/developer/.ros/rtabmap.db`

#### 3. Visualize the result
```bash
$ export ROS_NAMESPACE=rtabmap
$ rosrun rtabmap_viz rtabmap_viz _frame_id:=base_link
```