#ifndef EXTENDED_CAMERA_LOADER_CAMERA_LOADER_H
#define EXTENDED_CAMERA_LOADER_CAMERA_LOADER_H

#include <rclcpp/rclcpp.hpp>

#include <extended_image_geometry_msgs/msg/extended_camera_info.hpp>
#include <extended_camera_loader/camera.h>


template<typename T> bool getParam(const rclcpp::Node::SharedPtr node, const std::string& key, T& var) {
  if (!node->get_parameter(key, var)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not get parameter '" << node->get_namespace() << "/" << key << "'");
    return false;
  } else {
    return true;
  }
}

namespace extended_image_geometry {

class CameraLoader {
public:
  CameraLoader(const rclcpp::Node::SharedPtr node);
  bool loadCameras();
  bool waitForCameraInfo(const rclcpp::Duration& timeout = rclcpp::Duration(0, 0)) const;
  bool cameraInfosReceived() const;
  bool waitForImages(const rclcpp::Duration& timeout = rclcpp::Duration(0, 0)) const;
  bool imagesReceived() const;

  void startImageSubscribers();
  void stopImageSubscribers();

  const std::vector<CameraPtr>& cameras() const;
private:
  rclcpp::Node::SharedPtr node_;
  std::vector<CameraPtr> cameras_;
};

}

#endif
