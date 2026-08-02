// Based on kalibr aslam_cv https://github.com/ethz-asl/kalibr/tree/master
#ifndef EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_H
#define EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_H

#include <eigen3/Eigen/Eigen>
#include <extended_camera_model/cameras/camera_geometry_base.h>

namespace extended_image_geometry {
namespace cameras {

template<typename PROJECTION_TYPE, typename SHUTTER_TYPE, typename MASK_TYPE>
class CameraGeometry : public CameraGeometryBase {
 public:
  CameraGeometry(const PROJECTION_TYPE& projection, const SHUTTER_TYPE& shutter, const MASK_TYPE& mask)
    : projection_(projection), shutter_(shutter), mask_(mask) {}

  bool vsEuclideanToKeypoint(const Eigen::Vector3d & p, Eigen::VectorXd & outKeypoint) const {
    bool valid = projection_.euclideanToKeypoint(p, outKeypoint);
    return valid && mask_.isValid(outKeypoint);
  }
  
  PROJECTION_TYPE projection_;
  SHUTTER_TYPE shutter_;
  MASK_TYPE mask_;
};

} // namespace cameras
} // namespace kalibr_image_geometry

#endif // EXTENDED_CAMERA_MODEL_CAMERAS_CAMERA_GEOMETRY_H
