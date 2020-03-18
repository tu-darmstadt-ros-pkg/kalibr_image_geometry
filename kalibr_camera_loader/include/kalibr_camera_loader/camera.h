#ifndef KALIBR_CAMERA_LOADER_CAMERA_H
#define KALIBR_CAMERA_LOADER_CAMERA_H

#include <kalibr_camera_model/camera_model.h>
#include <kalibr_image_geometry_msgs/ExtendedCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace kalibr_image_geometry {

class Camera {
public:
  Camera(const ros::NodeHandle& camera_nh);
  bool waitForCameraInfo(const ros::Duration& timeout) const;
  bool cameraInfoReceived() const;

  void startImageSubscriber();
  void stopImageSubscriber();

  std::string getCameraNs() const;
  sensor_msgs::ImageConstPtr getLastImage() const;
  cv_bridge::CvImageConstPtr getLastImageCv() const;

  std::string getName() const;
  const CameraModel& model() const;

private:
  void extendedCameraInfoCb(const kalibr_image_geometry_msgs::ExtendedCameraInfoConstPtr& camera_info);
  void cameraInfoCb(const sensor_msgs::CameraInfo& camera_info);
  void imageCb(const sensor_msgs::ImageConstPtr& image);

  ros::NodeHandle camera_nh_;
  std::string name_;
  CameraModel model_;
  sensor_msgs::ImageConstPtr last_image_;
  mutable cv_bridge::CvImageConstPtr last_image_cv_;

  ros::Subscriber extended_camera_info_sub_;
  ros::Subscriber camera_info_sub_;
  bool camera_info_received_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

typedef std::shared_ptr<Camera> CameraPtr ;

}

#endif
