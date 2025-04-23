#include <extended_camera_info_publisher/camera_info_publisher.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace extended_image_geometry {

CameraInfoPublisher::CameraInfoPublisher(const rclcpp::NodeOptions& options)
  : Node("extended_camera_info_publisher", options)
  {
  rcl_interfaces::msg::ParameterDescriptor camera_ns_options;
  camera_ns_options.description = "Namespace of the camera in which the camera info will be published";
  camera_ns_options.read_only = true;
  camera_ns_ = declare_parameter("camera_ns", "", camera_ns_options);

  declare_parameter("camera_name", "camera");
  declare_parameter("resolution", std::vector<int32_t>{640, 480});
  declare_parameter("frame_id", "camera_frame");
  declare_parameter("camera_model", "pinhole");
  declare_parameter("intrinsics", std::vector<double>{0.0, 0.0, 0.0, 0.0});
  declare_parameter("distortion_model", "none");
  declare_parameter("distortion_coeffs", std::vector<double>{0.0, 0.0, 0.0, 0.0});
  declare_parameter("mask_path", "");
}

bool CameraInfoPublisher::loadCameraInfoFromParam()
{
  camera_info_ = extended_image_geometry_msgs::msg::ExtendedCameraInfo();

  bool success = true;
  success = success && get_parameter("camera_name", camera_info_.camera_name);
  std::vector<int64_t> resolution_64;
  success = success && get_parameter("resolution", resolution_64);
  std::vector<int> resolution_32(resolution_64.begin(), resolution_64.end());
  camera_info_.resolution = resolution_32;
  success = success && get_parameter("frame_id", camera_info_.frame_id);
  camera_info_.header.frame_id = camera_info_.frame_id;
  success = success && get_parameter("camera_model", camera_info_.camera_model);
  success = success && get_parameter("intrinsics", camera_info_.intrinsics);
  success = success && get_parameter("distortion_model", camera_info_.distortion_model);
  success = success && get_parameter("distortion_coeffs", camera_info_.distortion_coeffs);



  // Load mask
  std::string mask_path;
  if (get_parameter("mask_path", mask_path)) {
    cv::Mat mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to load mask from '" << mask_path << "'.");
    } else {
      cv_bridge::CvImage cv_image;
      cv_image.encoding = "mono8";
      cv_image.image = mask;
      cv_image.toImageMsg(camera_info_.mask);
    }
  }

  if (success) {
    rclcpp::QoS qos_profile(10);
    qos_profile.transient_local();
    cam_info_pub_ = create_publisher<extended_image_geometry_msgs::msg::ExtendedCameraInfo>("extended_camera_info", qos_profile);
  }

  return success;
}

void CameraInfoPublisher::latchCameraInfo() {
  cam_info_pub_->publish(camera_info_);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(extended_image_geometry::CameraInfoPublisher)
