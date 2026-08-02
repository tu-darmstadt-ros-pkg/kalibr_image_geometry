#include <extended_camera_loader/camera_loader.h>

namespace extended_image_geometry {

CameraLoader::CameraLoader(const rclcpp::Node::SharedPtr node)
  : node_(node)
{
  node_->declare_parameter("cameras", std::vector<std::string>{});
  loadCameras();
}

bool CameraLoader::loadCameras()
{
  std::vector<std::string> camera_namespaces;
  node_->get_parameter("cameras", camera_namespaces);
  for (const std::string& ns: camera_namespaces) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Loading camera '" << ns << "'.");
    
    CameraPtr camera = std::make_shared<Camera>(node_, ns);
    //(node_->create_sub_node(ns));
    cameras_.push_back(camera);
  }
  return true;
}

bool CameraLoader::waitForCameraInfo(const rclcpp::Duration& timeout) const
{
  rclcpp::Rate rate(10);
  rclcpp::Time end = node_->get_clock()->now() + timeout;
  while (rclcpp::ok() &&
         (node_->get_clock()->now() < end || timeout.seconds() == 0.0)) {
    // Check if camera info of every cameras has been received
    if (cameraInfosReceived()) {
      return true;
    }
    rate.sleep();
    //rclcpp::spinOnce();
    rclcpp::spin_some(node_);
  }
  // Failed to receive all infos
  for (const CameraPtr& camera: cameras_) {
    if (!camera->cameraInfoReceived()) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Timed out waiting for camera info on topic '" << camera->getCameraNs() << "/extended_camera_info'");
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

bool CameraLoader::waitForImages(const rclcpp::Duration& timeout) const
{
  rclcpp::Rate rate(10);
  rclcpp::Time end = node_->get_clock()->now() + timeout;
  while (rclcpp::ok() &&
         (node_->get_clock()->now() < end || timeout.seconds() == 0.0)) {
    // Check if an image has been received by each camera
    if (imagesReceived()) {
      return true;
    }
    rate.sleep();
    //rclcpp::spinOnce();
    spin_some(node_);
  }
  // Failed to receive all infos
  for (const CameraPtr& camera: cameras_) {
    if (!camera->getLastImage()) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Timed out waiting for image on topic '" << camera->getCameraNs() << "/image_raw'");
    }
  }

  return false;
}

bool CameraLoader::imagesReceived() const
{
  for (const CameraPtr& camera: cameras_) {
    if (!camera->getLastImage()) {
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
