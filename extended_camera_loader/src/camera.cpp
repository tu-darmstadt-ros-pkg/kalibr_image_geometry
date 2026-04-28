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
    {"extended_camera_info_topic", "extended_camera_info"}, {"mask", ""}, {"type", "rgb"}});

  // Declare image transport outside of ns because it fails else
  node_->declare_parameter<std::string>(ns_ + ".image_transport", "raw");
  transport_hints_ = std::make_shared<image_transport::TransportHints>(node_.get(), "raw", ns_ + ".image_transport");

  // Allow to set initial camera info via parameters
  node_->declare_parameters<std::vector<double>>(ns_,  {{"intrinsics", std::vector<double>{}},
   {"distortion_coefficients", std::vector<double>{}}});
  node_->declare_parameters<std::string>(ns_, {{"distortion_model", ""},
   {"camera_model", ""},
   {"frame_id", ""}});
  node_->declare_parameters<std::vector<long int>>(ns_,  {{"resolution", std::vector<long int>{}}});

  // Load parameters
  node_->get_parameter(ns_ + ".image_topic", image_topic_);
  node_->get_parameter(ns_ + ".camera_info_topic", camera_info_topic_);
  node_->get_parameter(ns_ + ".extended_camera_info_topic", extended_camera_info_topic_);
  node_->get_parameter(ns_ + ".mask", mask_path_);
  node_->get_parameter(ns_ + ".type", type_);

  // for mono cameras, allow to load, min, max and color map
  if (type_ == "mono") {
    node_->declare_parameters<double>(ns_, {{"min_value", 0.0}, {"max_value", 65535.0}});
    node_->declare_parameter<std::string>(ns_ + ".color_map", "");
    node_->get_parameter(ns_ + ".min_value", min_value_);
    node_->get_parameter(ns_ + ".max_value", max_value_);
    std::string color_map_string;
    node_->get_parameter(ns_ + ".color_map", color_map_string);

    if (!color_map_string.empty()) {
      use_color_map_ = true;
      color_map_ = getColormapType(color_map_string);
    } else {
      use_color_map_ = false;
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Camera '" << ns_ << "' is set to mono mode with min_value: " << min_value_ << ", max_value: " << max_value_ << ", color_map: " << (use_color_map_ ? color_map_string : "none"));
  }

  // Load initial camera info from parameters, if available
  loadCameraInfoFromParams();

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
  image_sub_ = it_.subscribe(ns_ + "/" + image_topic_, 10, &Camera::imageCb, this, transport_hints_.get(), rclcpp::SubscriptionOptions());
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
  std::unique_lock lock(camera_info_mutex_);
  if (!cameraInfoReceived()) {
    // Overwrite mask if one was set manually
    if (!mask_msg_->data.empty()) {
      extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr info_copy(camera_info);
      info_copy->mask = *mask_msg_;
      model_.fromExtendedCameraInfo(info_copy);
    } else {
      model_.fromExtendedCameraInfo(camera_info);
    }
    extended_camera_info_received_ = true;
  } else if (camera_info_received_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, "Received extended camera info, after camera model has been initialized with standard camera info. This indicates a race condition! "
                         "Do not publish on extended_camera_info and camera_info at the same time. This message is throttled (1s).");
  }
}

void Camera::cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  std::unique_lock lock(camera_info_mutex_);
  if (!cameraInfoReceived()) {
    model_.fromCameraInfo(camera_info, mask_msg_);
    camera_info_received_ = true;
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
      if (use_color_map_) {
        auto last_image = getLastImage();
        auto cv_ptr = cv_bridge::toCvCopy(last_image, last_image->encoding);
        cv::Mat color_mapped;
        applyColorMapRanged(cv_ptr->image, color_mapped, color_map_, min_value_, max_value_);
        last_image_cv_ = std::make_shared<cv_bridge::CvImage>(cv_ptr->header, "rgb8", color_mapped);
      } else {
        last_image_cv_ = cv_bridge::toCvCopy(getLastImage(), "rgb8");
      }
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
  if (!model_.cameraInfo()) {
    RCLCPP_ERROR(node_->get_logger(), "Camera model is not initialized, cannot get camera name.");
    return "";
  }
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

builtin_interfaces::msg::Time Camera::getLastStamp() const
{
  if (!last_image_) {
    return builtin_interfaces::msg::Time();
  }
  return last_image_->header.stamp;
}

void Camera::loadCameraInfoFromParams()
{
  std::vector<double> intrinsics;
  node_->get_parameter(ns_ + ".intrinsics", intrinsics);
  std::vector<double> distortion_coefficients;
  node_->get_parameter(ns_ + ".distortion_coefficients", distortion_coefficients);
  std::string distortion_model;
  node_->get_parameter(ns_ + ".distortion_model", distortion_model);
  std::string camera_model;
  node_->get_parameter(ns_ + ".camera_model", camera_model);
  std::string frame_id;
  node_->get_parameter(ns_ + ".frame_id", frame_id);
  std::vector<long int> resolution;
  std::vector<int> resolution_int;
  node_->get_parameter(ns_ + ".resolution", resolution);
  // check if resolution is in 32 bit limit
  if (resolution.size() == 2 && (resolution[0] < std::numeric_limits<int>::max() || resolution[1] < std::numeric_limits<int>::max())) {
    resolution_int.push_back(resolution[0]);
    resolution_int.push_back(resolution[1]);
  }
  if (!intrinsics.empty() && !distortion_coefficients.empty() && !distortion_model.empty() && !camera_model.empty() && !resolution.empty()) {
    std::shared_ptr<extended_image_geometry_msgs::msg::ExtendedCameraInfo> camera_info = std::make_shared<extended_image_geometry_msgs::msg::ExtendedCameraInfo>();
    camera_info->intrinsics = intrinsics;
    camera_info->distortion_coeffs = distortion_coefficients;
    camera_info->distortion_model = distortion_model;
    camera_info->camera_model = camera_model;
    camera_info->frame_id = frame_id;
    camera_info->resolution = resolution_int;
    model_.fromExtendedCameraInfo(camera_info);
    extended_camera_info_received_ = true;
  }

}

}
