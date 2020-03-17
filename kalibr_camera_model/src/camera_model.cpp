#include <kalibr_camera_model/camera_model.h>

namespace kalibr_image_geometry {

CameraModel::CameraModel()
  : initialized_(false) {

}

bool CameraModel::fromExtendedCameraInfo(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info)
{
  camera_info_ = camera_info;
  camera_geometry_ = createCameraGeometry(camera_info);
  if (!camera_geometry_) {
    ROS_ERROR_STREAM("Creating camera geometry failed.");
    return false;
  }

  initialized_ = true;
  return true;
}

bool CameraModel::fromCameraInfo(const sensor_msgs::CameraInfo& camera_info)
{
  kalibr_image_geometry_msgs::ExtendedCameraInfo extended_camera_info;
  extended_camera_info.header = camera_info.header;
  extended_camera_info.camera_name = camera_info.header.frame_id;
  extended_camera_info.resolution[0] = static_cast<int>(camera_info.width);
  extended_camera_info.resolution[1] = static_cast<int>(camera_info.height);

  // Camera model
  extended_camera_info.camera_model = "pinhole";
  extended_camera_info.intrinsics.resize(4);
  extended_camera_info.intrinsics[0] = camera_info.K[0]; // fu
  extended_camera_info.intrinsics[1] = camera_info.K[4]; // fv
  extended_camera_info.intrinsics[2] = camera_info.K[2]; // pu
  extended_camera_info.intrinsics[3] = camera_info.K[5]; // pv

  // Distortion model
  if (camera_info.distortion_model == "" || camera_info.distortion_model == "none") {
    extended_camera_info.distortion_model = "none";
  } else if (camera_info.distortion_model == "plumb_bob") {
    bool zeros_or_empty = std::all_of(camera_info.D.begin(), camera_info.D.end(), [](int i) { return i==0; });
    if (zeros_or_empty) {
      extended_camera_info.distortion_model = "none";
    } else {
      if (camera_info.D.size() < 4) {
        ROS_ERROR_STREAM("Invalid number of distortion coefficients.");
        extended_camera_info.distortion_model = "none";
      } else {
        extended_camera_info.distortion_model = "radtan";
        extended_camera_info.distortion_coeffs.resize(4);
        std::copy(camera_info.D.begin(), camera_info.D.end(), extended_camera_info.distortion_coeffs.begin());
      }
    }
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
    ROS_ERROR_STREAM("Camera geometry has not been initialized yet.");
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

std::shared_ptr<CameraGeometryBase> CameraModel::createCameraGeometry(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info)
{

  // Load projection model
  if (camera_info.distortion_model == "radtan") {
    RadialTangentialDistortion distortion(camera_info.distortion_coeffs[0], camera_info.distortion_coeffs[1],
                                          camera_info.distortion_coeffs[2], camera_info.distortion_coeffs[3]);
    return createCameraGeometry(camera_info, distortion);
  } else if (camera_info.distortion_model == "none") {
    NoDistortion distortion;
    return createCameraGeometry(camera_info, distortion);
  }
  // Unknown distortion model
  ROS_ERROR_STREAM("Unknown distortion model '" << camera_info.distortion_model << "'");
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
  return std::pow(pixel(0) - camera_info_.resolution[0] / 2.0, 2) + std::pow(pixel(1) - camera_info_.resolution[1] / 2.0, 2);
}

const kalibr_image_geometry_msgs::ExtendedCameraInfo& CameraModel::cameraInfo() const
{
  return camera_info_;
}

}
