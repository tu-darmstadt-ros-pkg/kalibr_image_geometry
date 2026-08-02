// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_BASE_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_BASE_H

#include <eigen3/Eigen/Eigen>

namespace extended_image_geometry {
namespace cameras {

class CameraGeometryBase {
  public:
    virtual bool vsEuclideanToKeypoint(const Eigen::Vector3d& p, Eigen::VectorXd& outKeypoint) const = 0;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_BASE_H
