// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
// https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras/include/aslam/cameras/ExtendedUnifiedProjection.hpp
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_EXTENDED_UNIFIED_PROJECTION_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_EXTENDED_UNIFIED_PROJECTION_H

#include <eigen3/Eigen/Eigen>
#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {

namespace cameras {

template<typename DISTORTION_TYPE>
class ExtendedUnifiedProjection {
  public:

  enum {
    KeypointDimension = 2
  };

  ExtendedUnifiedProjection(double alpha, double beta, double fu, double fv, double cu, double cv, int ru, int rv, const DISTORTION_TYPE& distortion)
    : alpha_(alpha), beta_(beta), fu_(fu), fv_(fv), cu_(cu), cv_(cv), ru_(ru), rv_(rv), distortion_(distortion) {
    updateTemporaries();
  }

  void updateTemporaries() {
    fov_parameter_ = (alpha_ <= 0.5) ? alpha_ / (1 - alpha_) : (1 - alpha_) / alpha_;
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

    const double& x = p[0];
    const double& y = p[1];
    const double& z = p[2];

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;

    const double d2 = beta_ * (xx + yy) + zz;
    const double d = std::sqrt(d2);

    // Check if point will lead to a valid projection
    if (z <= -(fov_parameter_ * d))
    return false;

    double norm = alpha_ * d + (1 - alpha_) * z;
    double norm_inv = 1.0 / norm;

    outKeypoint[0] = x * norm_inv;
    outKeypoint[1] = y * norm_inv;

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

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

    // project the point

    const double& x = p[0];
    const double& y = p[1];
    const double& z = p[2];

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;

    double d2 = beta_ * (xx + yy) + zz;
    double d = std::sqrt(d2);
    double d_inv = 1.0/d;

      // Check if point will lead to a valid projection
    if (z <= -(fov_parameter_ * d))
    return false;

    double norm = alpha_ * d + (1 - alpha_) * z;
    double norm_inv = 1.0 / norm;

    outKeypoint[0] = p[0] * norm_inv;
    outKeypoint[1] = p[1] * norm_inv;

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    double denom = norm_inv * norm_inv * d_inv;
    double mid = -(alpha_ * beta_ * x * y) * denom;
    double add = norm * d;
    double addz = (alpha_ * z + (1 - alpha_) * d);


    J(0, 0) = fu_ * (add - x * x * alpha_ * beta_) * denom;
    J(1, 0) = fv_ * mid;
    
    J(0, 1) = fu_ * mid;
    J(1, 1) = fv_ * (add - y * y * alpha_ * beta_) * denom;
    
    J(0, 2) = -fu_ * x * addz * denom;
    J(1, 2) = -fv_ * y * addz * denom;

    return isValid(outKeypoint);
  }

  bool isValid(const Eigen::Vector2d & keypoint) const {
    return keypoint(0) >= 0 && keypoint(0) < ru_ && keypoint(1) >= 0
           && keypoint(1) < rv_;
  }

  private:
   double alpha_, beta_, fu_, fv_, cu_, cv_;
   int ru_, rv_;
   DISTORTION_TYPE distortion_;
   double fov_parameter_;
};

} // namespace cameras

} // namespace extended_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_EXTENDED_UNIFIED_PROJECTION_H
