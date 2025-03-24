#ifndef EXTENDED_CAMERA_INFO_PUBLISHER_CAMERA_INFO_PUBLISHER_H
#define EXTENDED_CAMERA_INFO_PUBLISHER_CAMERA_INFO_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <extended_image_geometry_msgs/msg/extended_camera_info.hpp>

namespace kalibr_image_geometry {

class CameraInfoPublisher : public rclcpp::Node {
public:
  CameraInfoPublisher();
  bool loadCameraInfoFromYAML(const std::string& file_path);
  void latchCameraInfo();
private:
  std::string camera_ns_;
  extended_image_geometry_msgs::msg::ExtendedCameraInfo camera_info_;
  rclcpp::Publisher<extended_image_geometry_msgs::msg::ExtendedCameraInfo>::SharedPtr cam_info_pub_;
};

}

#endif
