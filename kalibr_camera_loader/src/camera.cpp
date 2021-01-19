#include <kalibr_camera_loader/camera.h>

namespace kalibr_image_geometry {

Camera::Camera(const ros::NodeHandle& camera_nh)
  : camera_nh_(camera_nh), camera_info_received_(false), extended_camera_info_received_(false), it_(camera_nh)
{
  // Load parameters
  camera_nh_.param<std::string>("image_topic", image_topic_, "image_raw");
  camera_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "camera_info");
  camera_nh_.param<std::string>("extended_camera_info_topic", extended_camera_info_topic_, "extended_camera_info");

  // Subscribers
  extended_camera_info_sub_ = camera_nh_.subscribe(extended_camera_info_topic_, 10, &Camera::extendedCameraInfoCb, this);
  camera_info_sub_ = camera_nh_.subscribe(camera_info_topic_, 10, &Camera::cameraInfoCb, this);
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
  return camera_info_received_ || extended_camera_info_received_;
}

void Camera::startImageSubscriber()
{
  image_sub_ = it_.subscribe(image_topic_, 10, &Camera::imageCb, this);
}

void Camera::stopImageSubscriber()
{
  image_sub_.shutdown();
}

std::string Camera::getCameraNs() const
{
  return camera_nh_.getNamespace();
}

void Camera::extendedCameraInfoCb(const kalibr_image_geometry_msgs::ExtendedCameraInfoConstPtr& camera_info)
{
  if (!cameraInfoReceived()) {
    extended_camera_info_received_ = true;
    model_.fromExtendedCameraInfo(*camera_info);
  } else if (camera_info_received_) {
    ROS_WARN_THROTTLE(1, "Received extended camera info, after camera model has been initialized with standard camera info. This indicates a race condition! "
                         "Do not publish on extended_camera_info and camera_info at the same time. This message is throttled (1s).");
  }
}

void Camera::cameraInfoCb(const sensor_msgs::CameraInfo& camera_info)
{
  if (!cameraInfoReceived()) {
    camera_info_received_ = true;
    model_.fromCameraInfo(camera_info);
  } else if (extended_camera_info_received_) {
    ROS_WARN_THROTTLE(1, "Received standard camera info, after camera model has been initialized with extended camera info. This indicates a race condition! "
                         "Do not publish on extended_camera_info and camera_info at the same time. This message is throttled (1s).");
  }
}

void Camera::imageCb(const sensor_msgs::ImageConstPtr& image)
{
  last_image_ = image;
  last_image_cv_.reset();
}

cv_bridge::CvImageConstPtr Camera::getLastImageCv() const
{
  // Check if value was cached
  if (!last_image_cv_ && last_image_) {
    try
    {
      last_image_cv_ = cv_bridge::toCvCopy(getLastImage(), "rgb8");
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("CV Bridge conversion failed: " << e.what());
    }
  }

  return last_image_cv_;
}

std::string Camera::getName() const
{
  return model().cameraInfo().camera_name;
}

const CameraModel& Camera::model() const
{
  return model_;
}

sensor_msgs::ImageConstPtr Camera::getLastImage() const
{
  return last_image_;
}

}
