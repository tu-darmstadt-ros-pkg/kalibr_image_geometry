#include <kalibr_camera_loader/camera.h>

namespace kalibr_image_geometry {
namespace kalibr_camera_loader {

Camera::Camera(const ros::NodeHandle& camera_nh)
  : camera_nh_(camera_nh), camera_info_received_(false), it_(camera_nh)
{
  camera_info_sub_ = camera_nh_.subscribe("extended_camera_info", 10, &Camera::cameraInfoCb, this);
  startImageSubscriber();
}

bool Camera::waitForCameraInfo(const ros::Duration& timeout) const
{
  ros::Rate rate(10);
  ros::Time end = ros::Time::now() + timeout;
  if (ros::Time::now() < end) {
    if (cameraInfoReceived()) {
      return true;
    }
    rate.sleep();
    ros::spinOnce();
  }
  return cameraInfoReceived();
}

bool Camera::cameraInfoReceived() const
{
  return camera_info_received_;
}

void Camera::startImageSubscriber()
{
  image_sub_ = it_.subscribe("image_raw", 10, &Camera::imageCb, this);
}

void Camera::stopImageSubscriber()
{
  image_sub_.shutdown();
}

std::string Camera::getCameraNs() const
{
  return camera_nh_.getNamespace();
}

void Camera::cameraInfoCb(const kalibr_image_geometry_msgs::ExtendedCameraInfoConstPtr& camera_info)
{
  if (!cameraInfoReceived()) {
    model_.fromExtendedCameraInfo(*camera_info);
  }
  camera_info_received_ = true;
}

void Camera::imageCb(const sensor_msgs::ImageConstPtr& image)
{
  last_image_ = image;
}

cv_bridge::CvImageConstPtr Camera::getLastImageCv() const
{
  cv_bridge::CvImageConstPtr last_image_cv;
  try
  {
    last_image_cv = cv_bridge::toCvCopy(getLastImage(), "rgb8");
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("CV Bridge conversion failed: " << e.what());
  }
  return last_image_cv;
}

std::string Camera::getName() const
{
  return model().cameraInfo().camera_name;
}

const kalibr_camera_model::CameraModel& Camera::model() const
{
  return model_;
}

sensor_msgs::ImageConstPtr Camera::getLastImage() const
{
  return last_image_;
}

}

}