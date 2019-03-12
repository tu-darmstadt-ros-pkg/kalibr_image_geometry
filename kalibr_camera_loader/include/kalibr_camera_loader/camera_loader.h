#ifndef KALIBR_CAMERA_LOADER_CAMERA_LOADER_H
#define KALIBR_CAMERA_LOADER_CAMERA_LOADER_H

#include <ros/ros.h>

#include <kalibr_image_geometry_msgs/ExtendedCameraInfo.h>
#include <kalibr_camera_loader/camera.h>


template<typename T> bool getParam(const ros::NodeHandle& nh, const std::string& key, T& var) {
  if (!nh.getParam(key, var)) {
    ROS_ERROR_STREAM("Could not get parameter '" + nh.getNamespace() + "/" << key << "'");
    return false;
  } else {
    return true;
  }
}

namespace kalibr_image_geometry {

class CameraLoader {
public:
  CameraLoader(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  bool waitForCameraInfo(const ros::Duration& timeout = ros::Duration(0)) const;
  bool cameraInfosReceived() const;

  void startImageSubscribers();
  void stopImageSubscribers();

  const std::vector<CameraPtr>& cameras() const;
private:
  std::vector<CameraPtr> cameras_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};

}

#endif
