#include <kalibr_camera_loader/camera_loader.h>

namespace kalibr_image_geometry {

CameraLoader::CameraLoader(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  std::vector<std::string> camera_namespaces;
  getParam(pnh, "cameras", camera_namespaces);
  for (const std::string& ns: camera_namespaces) {
    ROS_INFO_STREAM("Loading camera '" << ns << "'.");
    cameras_.emplace_back(ns); // Construct camera in-place
  }
}

bool CameraLoader::waitForCameraInfo(const ros::Duration& timeout) const
{
  ros::Rate rate(10);
  ros::Time end = ros::Time::now() + timeout;
  while (ros::Time::now() < end) {
    // Check if camera info of every cameras has been received
    bool all_received = true;
    for (const Camera& camera: cameras_) {
      if (!camera.cameraInfoReceived()) {
        all_received = false;
        break;
      }
    }
    if (all_received) {
      return true;
    }
    rate.sleep();
    ros::spinOnce();
  }
  // Failed to receive all infos
  for (const Camera& camera: cameras_) {
    if (!camera.cameraInfoReceived()) {
      ROS_WARN_STREAM("Timed out waiting for camera info on topic '" << camera.getCameraNs() << "/extended_camera_info'");
    }
  }

  return false;
}

void CameraLoader::startImageSubscribers()
{
  for (Camera& camera: cameras_) {
    camera.startImageSubscriber();
  }
}

void CameraLoader::stopImageSubscribers()
{
  for (Camera& camera: cameras_) {
    camera.stopImageSubscriber();
  }
}

const std::vector<Camera>&CameraLoader::cameras() const
{
  return cameras_;
}

}
