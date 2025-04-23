// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_PINHOLE_PROJECTION_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_PINHOLE_PROJECTION_H

#include <eigen3/Eigen/Eigen>
#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {
namespace cameras {

template<typename DISTORTION_TYPE>
class PinholeProjection {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum {
    KeypointDimension = 2
  };
  PinholeProjection(double fu, double fv, double cu, double cv, int ru, int rv, const DISTORTION_TYPE& distortion)
    : fu_(fu), fv_(fv), cu_(cu), cv_(cv), ru_(ru), rv_(rv), distortion_(distortion) {}

  template<typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypointConst) const {

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
        DERIVED_K> &>(outKeypointConst);

    outKeypoint.derived().resize(2);
    double rz = 1.0 / p[2];
    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;

    distortion_.distort(outKeypoint);

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    return isValid(outKeypoint) && p[2] > 0;
  }

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypointConst,
      const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_P>, 3);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_K>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JP>, 2, 3);

    Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
        DERIVED_K> &>(outKeypointConst);
    outKeypoint.derived().resize(2);

    // Jacobian:
    Eigen::MatrixBase<DERIVED_JP> & J =
        const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
    J.derived().resize(KeypointDimension, 3);
    J.setZero();

    double rz = 1.0 / p[2];
    double rz2 = rz * rz;

    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;

    Eigen::MatrixXd Jd;
    distortion_.distort(outKeypoint, Jd);  // distort and Jacobian wrt. keypoint

    // Jacobian including distortion
    J(0, 0) = fu_ * Jd(0, 0) * rz;
    J(0, 1) = fu_ * Jd(0, 1) * rz;
    J(0, 2) = -fu_ * (p[0] * Jd(0, 0) + p[1] * Jd(0, 1)) * rz2;
    J(1, 0) = fv_ * Jd(1, 0) * rz;
    J(1, 1) = fv_ * Jd(1, 1) * rz;
    J(1, 2) = -fv_ * (p[0] * Jd(1, 0) + p[1] * Jd(1, 1)) * rz2;

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    return isValid(outKeypoint) && p[2] > 0;
  }

  int keypointDimension() const {
    return KeypointDimension;
  }

  template<typename DERIVED_K>
  bool isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
    return keypoint[0] >= 0 && keypoint[1] >= 0 && keypoint[0] < (double) ru_ && keypoint[1] < (double) rv_;
  }

 private:
  double fu_, fv_, cu_, cv_;
  int ru_, rv_;
  DISTORTION_TYPE distortion_;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_PINHOLE_PROJECTION_H
