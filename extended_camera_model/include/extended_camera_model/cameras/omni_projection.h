// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_OMNI_PROJECTION_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_OMNI_PROJECTION_H

#include <eigen3/Eigen/Eigen>
#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {
namespace cameras {

template<typename DISTORTION_TYPE>
class OmniProjection {
 public:

  enum {
    KeypointDimension = 2
  };

  OmniProjection(double xi, double fu, double fv, double cu, double cv, int ru, int rv, const DISTORTION_TYPE& distortion) :
  xi_(xi), fu_(fu), fv_(fv), cu_(cu), cv_(cv), ru_(ru), rv_(rv), distortion_(distortion) {
    // TODO was not correctly updated before
    fov_parameter_ = (xi_ <= 1.0) ? xi_ : 1 / xi_;
  }
  
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
    //    SM_OUT(p.transpose());
    double d = p.norm();

    // Check if point will lead to a valid projection
    if (p[2] <= -(fov_parameter_ * d))
      return false;

    double rz = 1.0 / (p[2] + xi_ * d);
    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;

    distortion_.distort(outKeypoint);
    
    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    // Check if keypoint lies on the sensor
    return isValid(outKeypoint);
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

    double d = p.norm();

    // Check if point will lead to a valid projection
    if (p[2] <= -(fov_parameter_ * d))
      return false;

    // project the point
    double rz = 1.0 / (p[2] + xi_ * d);
    outKeypoint[0] = p[0] * rz;
    outKeypoint[1] = p[1] * rz;

    // Calculate jacobian
    rz = rz * rz / d;
    J(0, 0) = rz * (d * p[2] + xi_ * (p[1] * p[1] + p[2] * p[2]));
    J(1, 0) = -rz * xi_ * p[0] * p[1];
    J(0, 1) = J(1, 0);
    J(1, 1) = rz * (d * p[2] + xi_ * (p[0] * p[0] + p[2] * p[2]));
    rz = rz * (-xi_ * p[2] - d);
    J(0, 2) = p[0] * rz;
    J(1, 2) = p[1] * rz;

    Eigen::Matrix2d Jd;
    distortion_.distort(outKeypoint, Jd);

    rz = fu_ * (J(0, 0) * Jd(0, 0) + J(1, 0) * Jd(0, 1));
    J(1, 0) = fv_ * (J(0, 0) * Jd(1, 0) + J(1, 0) * Jd(1, 1));
    J(0, 0) = rz;

    rz = fu_ * (J(0, 1) * Jd(0, 0) + J(1, 1) * Jd(0, 1));
    J(1, 1) = fv_ * (J(0, 1) * Jd(1, 0) + J(1, 1) * Jd(1, 1));
    J(0, 1) = rz;

    rz = fu_ * (J(0, 2) * Jd(0, 0) + J(1, 2) * Jd(0, 1));
    J(1, 2) = fv_ * (J(0, 2) * Jd(1, 0) + J(1, 2) * Jd(1, 1));
    J(0, 2) = rz;

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    return isValid(outKeypoint);
  }
  
  int keypointDimension() const {
    return KeypointDimension;
  }

  template<typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
    
    return keypoint(0) >= 0 && keypoint(0) < ru_ && keypoint(1) >= 0
    && keypoint(1) < rv_;
  }

 private:
  double xi_, fu_, fv_, cu_, cv_;
  int ru_, rv_;
  DISTORTION_TYPE distortion_;
  double fov_parameter_;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_OMNI_PROJECTION_H
