#include <extended_camera_loader/camera.h>
#include <opencv2/highgui.hpp>

namespace extended_image_geometry {

Camera::Camera(const rclcpp::Node::SharedPtr node, std::string ns)
  : node_(node), ns_(ns), camera_info_received_(false), extended_camera_info_received_(false), it_(node)
{
  mask_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  // Declare parameters
  node_->declare_parameters<std::string>(ns_, 
    {{"image_topic", "image_raw"}, 
    {"camera_info_topic", "camera_info"},
    {"extended_camera_info_topic", "extended_camera_info"}, {"mask", ""}});

  // Load parameters
  node_->get_parameter(ns_ + ".image_topic", image_topic_);
  node_->get_parameter(ns_ + ".camera_info_topic", camera_info_topic_);
  node_->get_parameter(ns_ + ".extended_camera_info_topic", extended_camera_info_topic_);
  node_->get_parameter(ns_ + ".mask", mask_path_);

  // Load mask, if available
  if (!mask_path_.empty()) {
    cv::Mat mask = cv::imread(mask_path_, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to load mask from '" << mask_path_ << "'.");
    } else {
      cv_bridge::CvImage cv_image;
      cv_image.encoding = sensor_msgs::image_encodings::MONO8;
      cv_image.image = mask;
      mask_msg_ = cv_image.toImageMsg();
    }
  }

  // Subscribers
  rclcpp::QoS qos = rclcpp::QoS(10).transient_local();
  extended_camera_info_sub_ = node_->create_subscription<extended_image_geometry_msgs::msg::ExtendedCameraInfo>(
    ns_ + "/" + extended_camera_info_topic_, qos, std::bind(&Camera::extendedCameraInfoCb, this, std::placeholders::_1));
  
  camera_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    ns_ + "/" + camera_info_topic_, qos, std::bind(&Camera::cameraInfoCb, this, std::placeholders::_1));

}

bool Camera::waitForCameraInfo(const rclcpp::Duration& timeout) const
{
  rclcpp::Rate rate(10);
  rclcpp::Time end = node_->get_clock()->now() + timeout;
  if (node_->get_clock()->now() < end) {
    if (cameraInfoReceived()) {
      return true;
    }
    rate.sleep();
    //rclcpp::spinOnce();
    spin_some(node_);
  }
  return cameraInfoReceived();
}

bool Camera::cameraInfoReceived() const
{
  return camera_info_received_ || extended_camera_info_received_;
}

void Camera::startImageSubscriber()
{
  image_sub_ = it_.subscribe(ns_ + "/" + image_topic_, 10, std::bind(&Camera::imageCb, this, std::placeholders::_1));
}

void Camera::stopImageSubscriber()
{
  image_sub_.shutdown();
}

std::string Camera::getCameraNs() const
{
  return node_->get_namespace();
}

void Camera::extendedCameraInfoCb(const extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr camera_info)
{
  if (!cameraInfoReceived()) {
    extended_camera_info_received_ = true;
    // Overwrite mask if one was set manually
    if (!mask_msg_->data.empty()) {
      extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr info_copy(camera_info);
      info_copy->mask = *mask_msg_;
      model_.fromExtendedCameraInfo(info_copy);
    } else {
      model_.fromExtendedCameraInfo(camera_info);
    }
  } else if (camera_info_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, "Received extended camera info, after camera model has been initialized with standard camera info. This indicates a race condition! "
                         "Do not publish on extended_camera_info and camera_info at the same time. This message is throttled (1s).");
  }
}

void Camera::cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  if (!cameraInfoReceived()) {
    camera_info_received_ = true;
    model_.fromCameraInfo(camera_info, mask_msg_);
  } else if (extended_camera_info_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, "Received standard camera info, after camera model has been initialized with extended camera info. This indicates a race condition! "
                         "Do not publish on extended_camera_info and camera_info at the same time. This message is throttled (1s).");
  }
}

void Camera::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& image)
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
      RCLCPP_ERROR_STREAM(node_->get_logger(), "CV Bridge conversion failed: " << e.what());
    }
  }

  return last_image_cv_;
}

std::string Camera::getName() const
{
  return model().cameraInfo()->camera_name;
}

const CameraModel& Camera::model() const
{
  return model_;
}

std::shared_ptr<sensor_msgs::msg::Image const> Camera::getLastImage() const
{
  return last_image_;
}

}
