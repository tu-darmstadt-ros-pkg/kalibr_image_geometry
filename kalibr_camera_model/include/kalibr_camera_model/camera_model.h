#ifndef KALIBR_CAMERA_MODEL_CAMERA_MODEL_H
#define KALIBR_CAMERA_MODEL_CAMERA_MODEL_H

#include <ros/ros.h>
#include <kalibr_image_geometry_msgs/ExtendedCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Eigen>
#include <aslam/cameras.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace aslam::cameras;

namespace kalibr_image_geometry {

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
  bool fromCameraInfo(const sensor_msgs::CameraInfo& camera_info, const sensor_msgs::Image& mask=sensor_msgs::Image());
  bool isInitialized();

  bool worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const;
  Color worldToColor(const Eigen::Vector3d& point3d, const cv::Mat& img, double& confidence) const;
  double distanceFromCenter(Eigen::Vector2d& pixel) const;

  const kalibr_image_geometry_msgs::ExtendedCameraInfo& cameraInfo() const;

private:
  std::shared_ptr<CameraGeometryBase> createCameraGeometry(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info);

  template <typename T>
  std::shared_ptr<CameraGeometryBase> createCameraGeometry(const kalibr_image_geometry_msgs::ExtendedCameraInfo& camera_info, const T& distortion) {
    // Load mask
    cv::Mat mask;
    if (!camera_info.mask.data.empty()) {
      cv_bridge::CvImagePtr cv_image_ptr;
      try {
        cv_image_ptr = cv_bridge::toCvCopy(camera_info.mask);
        mask = cv_image_ptr->image;
      } catch (const cv_bridge::Exception& e) {
        ROS_ERROR_STREAM("Converting of mask to cv failed: " << e.what());
      }
    }

    ImageMask image_mask(mask);
    GlobalShutter global_shutter;

    if (camera_info.camera_model == "omni") {
      OmniProjection<T> projection(camera_info.intrinsics[0], camera_info.intrinsics[1], camera_info.intrinsics[2],
                                                            camera_info.intrinsics[3], camera_info.intrinsics[4], camera_info.resolution[0],
                                                            camera_info.resolution[1], distortion);
      return std::make_shared<CameraGeometry<OmniProjection<T>, GlobalShutter, ImageMask>>(projection, global_shutter, image_mask);
    } else if (camera_info.camera_model == "pinhole") {
      PinholeProjection<T> projection(camera_info.intrinsics[0], camera_info.intrinsics[1], camera_info.intrinsics[2], camera_info.intrinsics[3],
          camera_info.resolution[0], camera_info.resolution[1], distortion);
      return std::make_shared<CameraGeometry<PinholeProjection<T>, GlobalShutter, ImageMask>>(projection, global_shutter, image_mask);
    }
    // Unknown camera model
    ROS_ERROR_STREAM("Unknown camera model '" << camera_info.camera_model << "'");
    return std::shared_ptr<CameraGeometryBase>();
  }

  cv::Vec3b interpolate(const cv::Mat& img, const Eigen::Vector2d &pixel) const;

  bool initialized_;
  kalibr_image_geometry_msgs::ExtendedCameraInfo camera_info_;
  std::shared_ptr<CameraGeometryBase> camera_geometry_;

};

}

#endif
