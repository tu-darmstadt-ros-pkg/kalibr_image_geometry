#include <extended_camera_model/camera_model.h>

namespace extended_image_geometry {

CameraModel::CameraModel()
  : initialized_(false) {

}

bool CameraModel::fromExtendedCameraInfo(const extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr camera_info)
{
  camera_info_ = camera_info;
  camera_geometry_ = createCameraGeometry(camera_info);
  if (!camera_geometry_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Camera Model"), "Creating camera geometry failed.");
    return false;
  }

  initialized_ = true;
  return true;
}

bool CameraModel::fromCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info, const sensor_msgs::msg::Image::SharedPtr mask)
{
  extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr extended_camera_info = std::make_shared<extended_image_geometry_msgs::msg::ExtendedCameraInfo>();
  extended_camera_info->header = camera_info->header;
  extended_camera_info->camera_name = camera_info->header.frame_id;
  extended_camera_info->frame_id = camera_info->header.frame_id;
  extended_camera_info->resolution.resize(2);
  extended_camera_info->resolution[0] = static_cast<int>(camera_info->width);
  extended_camera_info->resolution[1] = static_cast<int>(camera_info->height);

  // Camera model
  extended_camera_info->camera_model = "pinhole";
  extended_camera_info->intrinsics.resize(4);
  extended_camera_info->intrinsics[0] = camera_info->k[0]; // fu
  extended_camera_info->intrinsics[1] = camera_info->k[4]; // fv
  extended_camera_info->intrinsics[2] = camera_info->k[2]; // pu
  extended_camera_info->intrinsics[3] = camera_info->k[5]; // pv

  // Distortion model
  if (camera_info->distortion_model.empty() || camera_info->distortion_model == "none") {
    extended_camera_info->distortion_model = "none";
  } else if (camera_info->distortion_model == "plumb_bob") {
    bool zeros_or_empty = std::all_of(camera_info->d.begin(), camera_info->d.end(), [](int i) { return i==0; });
    if (zeros_or_empty) {
      extended_camera_info->distortion_model = "none";
    } else {
      if (camera_info->d.size() < 4) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Camera Model"), "Invalid number of distortion coefficients.");
        extended_camera_info->distortion_model = "none";
      } else {
        extended_camera_info->distortion_model = "radtan";
        extended_camera_info->distortion_coeffs.resize(4);
        std::copy(camera_info->d.begin(), camera_info->d.end(), extended_camera_info->distortion_coeffs.begin());
      }
    }
  }

  // Copy mask, if it was given
  if (mask) {
    extended_camera_info->mask = *mask;
  }

  return fromExtendedCameraInfo(extended_camera_info);
}

bool CameraModel::isInitialized()
{
  return initialized_;
}

bool CameraModel::worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const
{
  if (!camera_geometry_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Camera Model"), "Camera geometry has not been initialized yet.");
    return false;
  }
  Eigen::VectorXd pixel(2);
  if (camera_geometry_->vsEuclideanToKeypoint(point3d, pixel)) {
    pixel_out = Eigen::Vector2d(pixel(0), pixel(1));
    return true;
  } else {
    pixel_out = Eigen::Vector2d::Zero();
    return false;
  }
}

Color CameraModel::worldToColor(const Eigen::Vector3d& point3d, const cv::Mat& img, double& confidence) const
{
  Eigen::Vector2d pixel(2);
  if (worldToPixel(point3d, pixel)) {
    cv::Vec3b color_vec = interpolate(img, pixel);
    confidence = distanceFromCenter(pixel);
    return Color(color_vec[0], color_vec[1], color_vec[2]);
  } else {
    confidence = INVALID; // kinda hacky?
    return Color();
  }
}

std::shared_ptr<CameraGeometryBase> CameraModel::createCameraGeometry(const extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr camera_info)
{

  // Load projection model
  if (camera_info->distortion_model == "radtan") {
    RadialTangentialDistortion distortion(camera_info->distortion_coeffs[0], camera_info->distortion_coeffs[1],
                                          camera_info->distortion_coeffs[2], camera_info->distortion_coeffs[3]);
    return createCameraGeometry(camera_info, distortion);
  } else if (camera_info->distortion_model == "none") {
    NoDistortion distortion;
    return createCameraGeometry(camera_info, distortion);
  }
  // Unknown distortion model
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("Camera Model"), "Unknown distortion model '" << camera_info->distortion_model << "'");
  return std::shared_ptr<CameraGeometryBase>();
}

cv::Vec3b CameraModel::interpolate(const cv::Mat& img, const Eigen::Vector2d& pixel) const
{
  cv::Point2f pt(static_cast<float>(pixel(0)), static_cast<float>(pixel(1)));
  cv::Mat patch;
  cv::getRectSubPix(img, cv::Size(1,1), pt, patch);
  return patch.at<cv::Vec3b>(0,0);
}

double CameraModel::distanceFromCenter(Eigen::Vector2d& pixel) const
{
  return std::pow(pixel(0) - camera_info_->resolution[0] / 2.0, 2) + std::pow(pixel(1) - camera_info_->resolution[1] / 2.0, 2);
}

const extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr CameraModel::cameraInfo() const
{
  return camera_info_;
}

}
