// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
// https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras/include/aslam/cameras/DoubleSphereProjection.hpp
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_DOUBLE_SPHERE_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_DOUBLE_SPHERE_H

#include <eigen3/Eigen/Eigen>
#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {

namespace cameras {

template<typename DISTORTION_TYPE>
class DoubleSphere {
  public:

    enum {
      KeypointDimension = 2
    };

  DoubleSphere(double xi, double alpha, double fu, double fv, double cu, double cv, int ru, int rv, const DISTORTION_TYPE& distortion)
    : xi_(xi), alpha_(alpha), fu_(fu), fv_(fv), cu_(cu), cv_(cv), ru_(ru), rv_(rv), distortion_(distortion) {
      updateTemporaries();
    }

  void updateTemporaries() {
    const double temp = alpha_ <= 0.5 ? alpha_ / (1 - alpha_) : (1 - alpha_) / alpha_;
    fov_parameter_ = (temp + xi_) / std::sqrt(2*temp * xi_ + xi_*xi_ + 1);
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

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;

    double r2 = xx + yy;

    double d1_2 = r2 + zz;
    double d1 = std::sqrt(d1_2);

    // Check if point will lead to a valid projection
    if (z <= -(fov_parameter_ * d1))
      return false;

    double k = xi_ * d1 + z;
    double kk = k * k;

    double d2_2 = r2 + kk;
    double d2 = std::sqrt(d2_2);

    double norm = alpha_ * d2 + (1 - alpha_) * k;
    double norm_inv = 1.0 / norm;

    outKeypoint[0] = x * norm_inv;
    outKeypoint[1] = y * norm_inv;

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

    // project the point

    const double& x = p[0];
    const double& y = p[1];
    const double& z = p[2];

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;

    double r2 = xx + yy;

    double d1_2 = r2 + zz;
    double d1 = std::sqrt(d1_2);
    double d1_inv = 1.0 / d1;

    // Check if point will lead to a valid projection
    if (z <= -(fov_parameter_ * d1))
      return false;

    double k = xi_ * d1 + z;
    double kk = k * k;

    double d2_2 = r2 + kk;
    double d2 = std::sqrt(d2_2);
    double d2_inv = 1.0 / d2;

    double norm = alpha_ * d2 + (1 - alpha_) * k;
    double norm_inv = 1.0 / norm;
    double norm_inv2 = norm_inv*norm_inv;

    outKeypoint[0] = x * norm_inv;
    outKeypoint[1] = y * norm_inv;

    outKeypoint[0] = fu_ * outKeypoint[0] + cu_;
    outKeypoint[1] = fv_ * outKeypoint[1] + cv_;

    double xy = x * y;
    double tt2 = xi_ * z * d1_inv + 1;

    double d_norm_d_r2 =
          (xi_ * (1 - alpha_) * d1_inv + alpha_ * (xi_ * k * d1_inv + 1) * d2_inv) *
          norm_inv2;

    double tmp2 = ((1 - alpha_) * tt2 + alpha_ * k * tt2 * d2_inv) * norm_inv2;

    J(0, 0) = fu_ * (norm_inv - xx * d_norm_d_r2);
    J(1, 0) = -fv_ * xy * d_norm_d_r2;

    J(0, 1) = -fu_ * xy * d_norm_d_r2;
    J(1, 1) = fv_ * (norm_inv - yy * d_norm_d_r2);

    J(0, 2) = -fu_ * x * tmp2;
    J(1, 2) = -fv_ * y * tmp2;


    return isValid(outKeypoint);
  }

  int keypointDimension() const {
    return KeypointDimension;
  }

  template<typename DERIVED_K>
  bool isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
    return keypoint(0) >= 0 && keypoint(0) < ru_ && keypoint(1) >= 0
    && keypoint(1) < rv_;
  }

  private:
  
    double xi_, alpha_, fu_, fv_, cu_, cv_;
    int ru_, rv_;
    double fov_parameter_;
    DISTORTION_TYPE distortion_;

};

} // namespace cameras

} // namespace extended_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_DOUBLE_SPHERE_H
