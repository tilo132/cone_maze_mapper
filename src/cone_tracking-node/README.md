# cone_tracking-node

## Overview
This package contains two nodes "cone_tracking" and "lidar_bbox"

-   Cone_Tracking:\
    subscribes to an CompressedImage topic and uses a Yolov8 network and visual trackers to find bounding boxes and ids for cones in the images.

-   Lidar_Bbox:\
    takes raw lidar data and the output of the cone_tracking node and calculates both 3d positions of the cones. It then creates a LaserScan message from the lidar data and adds in points for the calculated cones.


## Yolo weights
The Cone_Tracking node requires the Yolov8 weights file to be downloaded and placed in a path set by the "yolo_weights" parameter. The weights can be downloaded at https://seafile.zfn.uni-bremen.de/f/f19b0f3ea66b4938a539/?dl=1

## Camera to Lidar Calibration
The Lidar_Bbox node expects to find a tf transform from the camera frame to the lidar frame.

## Parameters

### Cone_Tracking

| Parameter | Type | Description |
|-----------|------|-------------|
| yolo_weights | string | Path to the Yolov8 weights file |
| tracker_lifetime | interger | The number of seconds a tracker will be kept alive without an update |
| tracking_match_distance_threshold | interger | The maximum distance between a new detection and a tracker for them to be considered a match |
| old_track_pos_hold_time | interger | number of frames old tracked positions are kept and taking into consideration when rerunning Yolo |
| overlay_bbox | bool | If true an image with the overlaid bounding boxes will be published |
| tracker_type | string | The tracker model to use. Options: KCF, CSRT, MIL, GOTURN, MOSSE |


### Lidar_Bbox
| Parameter | Type | Description |
|-----------|------|-------------|
| frame_camera | string | The camera frame |
| frame_lidar | string | The lidar frame |
| frame_base | string | The base frame |
| publish_pointcloud | bool | Whether to publish the pointcloud of the cone positions |
| publish_laserscan | bool | Whether to publish the laserscan |
| publish_transformed_pointcloud | bool | Whether to publish the cone pointcloud transofrmed to the base frame |
| publish_estimated_cones | bool | Whether to publish the estimated cone positions |2
| publish_filtered_pointcloud | bool | Whether to publish the filtered cone pointcloud where all points not close to cones are filtered out |
| time_synchronizer_buffer_size | interger | The buffer size of the time synchronizer |
| time_synchronizer_max_delay | interger | The maximum delay of the time synchronizer |
| laserscan_height_threshold | float | The z height below which points are considered to be floot |

## Installation

### Dependencies
-   [ROS Humble](http://docs.ros.org/en/humble/Installation.html)
-   [OpenCV](https://pypi.org/project/opencv-contrib-python/)
-   [Yolov8](https://pypi.org/project/ultralytics/)

### Building from Source

To build from source, clone the latest version from this repository into your workspace and compile the package using colcon build:
```
cd workspace/src
git clone git@gitlab.ips.biba.uni-bremen.de:ftf-navi2/cone_tracking-node.git
cd ../
colcon build --packages-up-to cone_tracking
```

