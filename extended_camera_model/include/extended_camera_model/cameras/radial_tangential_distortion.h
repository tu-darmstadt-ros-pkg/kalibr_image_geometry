// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_RADIAL_TANGENTIAL_DISTORTION_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_RADIAL_TANGENTIAL_DISTORTION_H

#include <extended_camera_model/cameras/static_asserts.h>

namespace extended_image_geometry {
namespace cameras {

class RadialTangentialDistortion {
 public:
  RadialTangentialDistortion(double k1, double k2, double p1, double p2)
    : k1_(k1), k2_(k2), p1_(p1), p2_(p2) {}

  template<typename DERIVED_Y>
  void distort(
      const Eigen::MatrixBase<DERIVED_Y> & yconst) const {

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);

    Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
    y.derived().resize(2);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = y[0] * y[0];
    my2_u = y[1] * y[1];
    mxy_u = y[0] * y[1];
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;
    y[0] += y[0] * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u);
    y[1] += y[1] * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);
  }

  template<typename DERIVED_Y, typename DERIVED_JY>
  void distort(
    const Eigen::MatrixBase<DERIVED_Y> & yconst,
    const Eigen::MatrixBase<DERIVED_JY> & outJy) const {

    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_Y>, 2);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
        Eigen::MatrixBase<DERIVED_JY>, 2, 2);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    Eigen::MatrixBase<DERIVED_JY> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JY> &>(outJy);
    J.derived().resize(2, 2);
    J.setZero();

    Eigen::MatrixBase<DERIVED_Y> & y =
      const_cast<Eigen::MatrixBase<DERIVED_Y> &>(yconst);
    y.derived().resize(2);

    mx2_u = y[0] * y[0];
    my2_u = y[1] * y[1];
    mxy_u = y[0] * y[1];
    rho2_u = mx2_u + my2_u;

    rad_dist_u = k1_ * rho2_u + k2_ * rho2_u * rho2_u;

    J(0, 0) = 1 + rad_dist_u + k1_ * 2.0 * mx2_u + k2_ * rho2_u * 4 * mx2_u
        + 2.0 * p1_ * y[1] + 6 * p2_ * y[0];
    J(1, 0) = k1_ * 2.0 * y[0] * y[1] + k2_ * 4 * rho2_u * y[0] * y[1]
        + p1_ * 2.0 * y[0] + 2.0 * p2_ * y[1];
    J(0, 1) = J(1, 0);
    J(1, 1) = 1 + rad_dist_u + k1_ * 2.0 * my2_u + k2_ * rho2_u * 4 * my2_u
        + 6 * p1_ * y[1] + 2.0 * p2_ * y[0];

    y[0] += y[0] * rad_dist_u + 2.0 * p1_ * mxy_u + p2_ * (rho2_u + 2.0 * mx2_u);
    y[1] += y[1] * rad_dist_u + 2.0 * p2_ * mxy_u + p1_ * (rho2_u + 2.0 * my2_u);
  }

 private:
  double k1_, k2_, p1_, p2_;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_RADIAL_TANGENTIAL_DISTORTION_H
