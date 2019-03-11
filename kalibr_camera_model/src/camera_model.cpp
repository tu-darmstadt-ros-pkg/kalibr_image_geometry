#include <kalibr_camera_model/camera_model.h>

#include <cv_bridge/cv_bridge.h>

namespace kalibr_image_geometry {
namespace kalibr_camera_model {

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

bool CameraModel::isInitialized()
{
  return initialized_;
}

bool CameraModel::worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const
{
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
  // Load mask
  cv_bridge::CvImagePtr cv_image_ptr;
  try {
    cv_image_ptr = cv_bridge::toCvCopy(camera_info.mask);
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("Converting of mask to cv failed: " << e.what());
  }
  ImageMask image_mask(cv_image_ptr->image);
  GlobalShutter global_shutter;

  // Load projection model
  std::shared_ptr<CameraGeometryBase> camera_geometry;
  if (camera_info.distortion_model == "radtan") {
    RadialTangentialDistortion distortion(camera_info.distortion_coeffs[0], camera_info.distortion_coeffs[1],
                                          camera_info.distortion_coeffs[2], camera_info.distortion_coeffs[3]);
    if (camera_info.camera_model == "omni") {
      OmniProjection<RadialTangentialDistortion> projection(camera_info.intrinsics[0], camera_info.intrinsics[1], camera_info.intrinsics[2],
                                                            camera_info.intrinsics[3], camera_info.intrinsics[4], camera_info.resolution[0],
                                                            camera_info.resolution[1], distortion);
      camera_geometry = std::make_shared<CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, ImageMask>>(projection, global_shutter, image_mask);
    }
  }

  return camera_geometry;
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
}
