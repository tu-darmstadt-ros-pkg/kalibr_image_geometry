// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_NO_DISTORTION_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_NO_DISTORTION_H

#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {
namespace cameras {

class NoDistortion {
 public:
  NoDistortion() {}

  template<typename DERIVED_Y>
  void distort(const Eigen::MatrixBase<DERIVED_Y> & /* y */) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
  }

  template<typename DERIVED_Y, typename DERIVED_JY>
  void distort(
      const Eigen::MatrixBase<DERIVED_Y> & /* y */,
      const Eigen::MatrixBase<DERIVED_JY> & outJyConst) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JY>, 2, 2);
    Eigen::MatrixBase<DERIVED_JY> & outJy = const_cast<Eigen::MatrixBase<
        DERIVED_JY> &>(outJyConst);
    outJy.derived().resize(2, 2);
    outJy.setIdentity();
  }
 private:
};


} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_NO_DISTORTION_H
