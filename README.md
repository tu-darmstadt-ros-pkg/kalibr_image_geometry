# kalibr_image_geometry
These [ROS](https://www.ros.org/) packages implement a pipeline to interpret images geometrically similar to [image_geometry](http://wiki.ros.org/image_geometry) for cameras calibrated with [kalibr](https://github.com/ethz-asl/kalibr). The calibration parameters are published as an [ExtendedCameraInfo.msg](https://github.com/tu-darmstadt-ros-pkg/kalibr_image_geometry/blob/master/kalibr_image_geometry_msgs/msg/ExtendedCameraInfo.msg).  Calibration information can also be loaded from standard [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html).
Notable packages, that use these libraries are image_projection and color_cloud_from_image. 
The author of this package is not affiliated with the authors of kalibr.
## Overview
### kalibr_camera_loader
Constructs a camera model from a yaml configuration file. By default, waits on topics camera_info and extended_camera_info (can be configured). Make sure, only one of these topics is published.
### kalibr_camera_model
Camera geometry calculations
### kalibr_extended_camera_info_publisher
Implements the camera_info_publisher_node node.
Publishes an ExtendedCameraInfo message as a latched topic. This node should be started for each camera
### kalibr_image_geometry_msgs

## Getting started
### Installation
### Basic Usage
