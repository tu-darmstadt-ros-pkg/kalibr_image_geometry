// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_IMAGE_MASK_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_IMAGE_MASK_H

#include <cv_bridge/cv_bridge.hpp>

namespace extended_image_geometry {
namespace cameras {

class ImageMask {
 public:
  ImageMask(const cv::Mat& mask, double scale = 1.0) : mask_(mask), scale_(scale) {}

  template<typename DERIVED>
  bool isValid(const Eigen::MatrixBase<DERIVED> & k) const {
    int k1 = k(1, 0) * scale_;
    int k0 = k(0, 0) * scale_;
    // \todo fix this when it is initialized properly
    return !mask_.data || (mask_.at<unsigned char>(k1, k0) > 0);
    //return true;
  }

 private:
  cv::Mat mask_;
  double scale_;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_IMAGE_MASK_H
