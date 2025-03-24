#include <kalibr_extended_camera_info_publisher/camera_info_publisher.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace kalibr_image_geometry {

CameraInfoPublisher::CameraInfoPublisher()
  : Node("extended_camera_info_publisher")
  {
  rcl_interfaces::msg::ParameterDescriptor camera_ns_options;
  camera_ns_options.description = "Namespace of the camera in which the camera info will be published";
  camera_ns_options.read_only = true;
  camera_ns_ = declare_parameter("camera_ns", "", camera_ns_options);
}

bool CameraInfoPublisher::loadCameraInfoFromYAML(const std::string& file_name)
{
  camera_info_ = kalibr_image_geometry_msgs::ExtendedCameraInfo();

  bool success = true;
  success = success && getParam<std::string>(nh, "camera_name", camera_info_.camera_name);
  success = success && getParam<std::vector<int>>(nh, "resolution", camera_info_.resolution);
  success = success && getParam<std::string>(nh, "frame_id", camera_info_.frame_id);
  camera_info_.header.frame_id = camera_info_.frame_id;
  success = success && getParam<std::string>(nh, "camera_model", camera_info_.camera_model);
  success = success && getParam<std::vector<double>>(nh, "intrinsics", camera_info_.intrinsics);
  success = success && getParam<std::string>(nh, "distortion_model", camera_info_.distortion_model);
  success = success && getParam<std::vector<double>>(nh, "distortion_coeffs", camera_info_.distortion_coeffs);

  // Load mask
  std::string mask_path;
  if (nh.getParam("mask_path", mask_path)) {
    cv::Mat mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
      ROS_ERROR_STREAM("Failed to load mask from '" << mask_path << "'.");
    } else {
      cv_bridge::CvImage cv_image;
      cv_image.encoding = "mono8";
      cv_image.image = mask;
      cv_image.toImageMsg(camera_info_.mask);
    }
  }

  if (success) {
    cam_info_pub_ = camera_nh_.advertise<kalibr_image_geometry_msgs::ExtendedCameraInfo>("extended_camera_info", 10, true);
  }

  return success;
}

void CameraInfoPublisher::latchCameraInfo()
{
  cam_info_pub_.publish(camera_info_);
}

}
