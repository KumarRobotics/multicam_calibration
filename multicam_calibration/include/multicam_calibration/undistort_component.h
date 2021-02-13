/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_CALIBRATION_UNDISTORT_COMPONENT_H
#define MULTICAM_CALIBRATION_UNDISTORT_COMPONENT_H

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace multicam_calibration {
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using ImageConstPtr = Image::ConstSharedPtr;
using CameraInfoConstPtr = CameraInfo::ConstSharedPtr;

class UndistortComponent : public rclcpp::Node {
 public:
  UndistortComponent(const rclcpp::NodeOptions &options);

 private:
  void imageCallback(const ImageConstPtr &img);
  void cameraInfoCallback(CameraInfoConstPtr camInfo);
  // ---------- variables
  rclcpp::Subscription<CameraInfo>::SharedPtr cameraInfoSub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr cameraInfoPub_;
  CameraInfo undistortedCameraInfo_;
  image_transport::Subscriber imageSub_;
  image_transport::Publisher imagePub_;
  double fovScale_{1.0};
  double balance_{0.0};
  bool ignoreWrongDistortion_{false};
  cv::Mat mapx_;
  cv::Mat mapy_;
};
}  // namespace multicam_calibration
#endif
