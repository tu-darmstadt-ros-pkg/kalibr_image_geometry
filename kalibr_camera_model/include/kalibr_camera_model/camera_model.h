#ifndef KALIBR_CAMERA_MODEL_CAMERA_MODEL_H
#define KALIBR_CAMERA_MODEL_CAMERA_MODEL_H

#include <ros/ros.h>
#include <kalibr_image_geometry_msgs/ExtendedCameraInfo.h>
#include <eigen3/Eigen/Eigen>
#include <aslam/cameras.hpp>

using namespace aslam::cameras;

namespace kalibr_image_geometry {
namespace kalibr_camera_model {

constexpr double INVALID = std::numeric_limits<double>::max();

struct Color {
  Color() : r(0), g(0), b(0) {}
  Color(uint8_t _r, uint8_t _g, uint8_t _b)
    : r(_r), g(_g), b(_b) {}

  uint8_t r, g, b;
};

class CameraModel {
public:
  CameraModel();
  bool fromExtendedCameraInfo(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info);

  bool worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const;
  Color worldToColor(const Eigen::Vector3d& point3d, const cv::Mat& img, double& confidence) const;

private:
  std::shared_ptr<CameraGeometryBase> createCameraGeometry(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info);
  cv::Vec3b interpolate(const cv::Mat& img, const Eigen::Vector2d &pixel) const;
  double distanceFromCenter(Eigen::Vector2d& pixel) const;

  kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info_;
  std::shared_ptr<CameraGeometryBase> camera_geometry_;

};

}
}

#endif
