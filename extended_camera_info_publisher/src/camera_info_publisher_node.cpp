#include <extended_camera_info_publisher/camera_info_publisher.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<extended_image_geometry::CameraInfoPublisher> cam_info_pub = std::make_shared<extended_image_geometry::CameraInfoPublisher>(rclcpp::NodeOptions());

  if (cam_info_pub->loadCameraInfoFromParam()) {
    cam_info_pub->latchCameraInfo();
  } else {
    RCLCPP_ERROR_STREAM(cam_info_pub->get_logger(), "Failed to load camera info.");
    return -1;
  }

  rclcpp::spin(cam_info_pub);
  rclcpp::shutdown();
  return 0;
}
