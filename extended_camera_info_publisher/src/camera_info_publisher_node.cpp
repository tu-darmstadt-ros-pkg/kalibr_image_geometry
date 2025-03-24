#include <kalibr_extended_camera_info_publisher/camera_info_publisher.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "extended_camera_info_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  kalibr_image_geometry::CameraInfoPublisher cam_info_pub(pnh);

  if (cam_info_pub.loadCameraInfoFromNamespace(pnh)) {
    cam_info_pub.latchCameraInfo();
  } else {
    ROS_ERROR_STREAM("Failed to load camera info.");
    return -1;
  }

  ros::spin();
  return 0;
}
