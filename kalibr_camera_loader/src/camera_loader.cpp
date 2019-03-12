#include <kalibr_camera_loader/camera_loader.h>

namespace kalibr_image_geometry {

CameraLoader::CameraLoader(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  std::vector<std::string> camera_namespaces;
  getParam(pnh, "cameras", camera_namespaces);
  for (const std::string& ns: camera_namespaces) {
    ROS_INFO_STREAM("Loading camera '" << ns << "'.");
    CameraPtr camera = std::make_shared<Camera>(ns);
    cameras_.push_back(camera);
  }
}

bool CameraLoader::waitForCameraInfo(const ros::Duration& timeout) const
{
  ros::Rate rate(10);
  ros::Time end = ros::Time::now() + timeout;
  while (ros::ok() &&
         (ros::Time::now() < end || timeout.toSec() == 0.0)) {
    // Check if camera info of every cameras has been received
    if (cameraInfosReceived()) {
      return true;
    }
    rate.sleep();
    ros::spinOnce();
  }
  // Failed to receive all infos
  for (const CameraPtr& camera: cameras_) {
    if (!camera->cameraInfoReceived()) {
      ROS_WARN_STREAM("Timed out waiting for camera info on topic '" << camera->getCameraNs() << "/extended_camera_info'");
    }
  }

  return false;
}

bool CameraLoader::cameraInfosReceived() const
{
  for (const CameraPtr& camera: cameras_) {
    if (!camera->cameraInfoReceived()) {
      return false;
    }
  }
  return true;
}

void CameraLoader::startImageSubscribers()
{
  for (CameraPtr& camera: cameras_) {
    camera->startImageSubscriber();
  }
}

void CameraLoader::stopImageSubscribers()
{
  for (CameraPtr& camera: cameras_) {
    camera->stopImageSubscriber();
  }
}

const std::vector<CameraPtr>& CameraLoader::cameras() const
{
  return cameras_;
}

}
