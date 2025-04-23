#ifndef EXTENDED_CAMERA_LOADER_CAMERA_H
#define EXTENDED_CAMERA_LOADER_CAMERA_H

#include <extended_camera_model/camera_model.h>
#include <extended_image_geometry_msgs/msg/extended_camera_info.hpp>
#include <sensor_msgs/msg/camera_info.h>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

namespace extended_image_geometry {

class Camera {
public:
  Camera(const rclcpp::Node::SharedPtr node, std::string ns);
  bool waitForCameraInfo(const rclcpp::Duration& timeout) const;
  bool cameraInfoReceived() const;

  void startImageSubscriber();
  void stopImageSubscriber();

  std::string getCameraNs() const;
  std::shared_ptr<sensor_msgs::msg::Image const> getLastImage() const;
  cv_bridge::CvImageConstPtr getLastImageCv() const;

  std::string getName() const;
  const CameraModel& model() const;

private:
  void extendedCameraInfoCb(const extended_image_geometry_msgs::msg::ExtendedCameraInfo::SharedPtr camera_info);
  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& image);

  rclcpp::Node::SharedPtr node_;
  std::string ns_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string extended_camera_info_topic_;
  std::string mask_path_;
  sensor_msgs::msg::Image::SharedPtr mask_msg_;

  std::string name_;
  CameraModel model_;
  std::shared_ptr<sensor_msgs::msg::Image const> last_image_;
  mutable cv_bridge::CvImage::ConstPtr last_image_cv_;

  rclcpp::Subscription<extended_image_geometry_msgs::msg::ExtendedCameraInfo>::SharedPtr extended_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  bool camera_info_received_;
  bool extended_camera_info_received_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

typedef std::shared_ptr<Camera> CameraPtr ;

}

#endif
