#ifndef KALIBR_EXTENDED_CAMERA_INFO_PUBLISHER_CAMERA_INFO_PUBLISHER_H
#define KALIBR_EXTENDED_CAMERA_INFO_PUBLISHER_CAMERA_INFO_PUBLISHER_H

#include <ros/ros.h>
#include <kalibr_image_geometry_msgs/ExtendedCameraInfo.h>

namespace kalibr_image_geometry {

template<typename T> bool getParam(const ros::NodeHandle& nh, const std::string& key, T& var) {
  if (!nh.getParam(key, var)) {
    ROS_ERROR_STREAM("Could not get parameter '" + nh.getNamespace() + "/" << key << "'");
    return false;
  } else {
    return true;
  }
}

class CameraInfoPublisher {
public:
  CameraInfoPublisher(const ros::NodeHandle& pnh);
  bool loadCameraInfoFromNamespace(const ros::NodeHandle& nh);
  void latchCameraInfo();
private:
  ros::NodeHandle pnh_;
  std::string camera_ns_;
  ros::NodeHandle camera_nh_;
  kalibr_image_geometry_msgs::ExtendedCameraInfo camera_info_;
  ros::Publisher cam_info_pub_;
};

}

#endif
