/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohtha
 */

#ifndef MULTICAM_CALIBRATION_CALIBRATION_TEST_COMPONENT_H
#define MULTICAM_CALIBRATION_CALIBRATION_TEST_COMPONENT_H

#include "multicam_calibration/calibration_data.h"
#include "multicam_calibration/multicam_apriltag_detector.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <iostream>
#include <memory>

namespace multicam_calibration {
using sensor_msgs::msg::CameraInfo;
using CameraInfoConstPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using ImageMsg = sensor_msgs::msg::Image;

typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>
    ApproxSyncPolicy2;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                        ImageMsg>
    ApproxSyncPolicy3;
typedef message_filters::Synchronizer<ApproxSyncPolicy2>
    ApproxTimeSynchronizer2;
typedef message_filters::Synchronizer<ApproxSyncPolicy3>
    ApproxTimeSynchronizer3;
typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg>
    ExactSynchronizer2;
typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg, ImageMsg>
    ExactSynchronizer3;

class CalibrationTestComponent : public rclcpp::Node {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CalibrationTestComponent(const rclcpp::NodeOptions &options);

  struct TestCamera : public CalibrationData {
    TestCamera(const CalibrationData &cd) : CalibrationData(cd){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void camInfoCallback(CameraInfoConstPtr caminfo);
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSub_;
    bool useCameraInfo_{true};
  };

 private:
  void callback1(ImageConstPtr const &img0);
  void callback2(ImageConstPtr const &img0, ImageConstPtr const &img1);
  void callback3(ImageConstPtr const &img0, ImageConstPtr const &img1,
                 ImageConstPtr const &img2);
  void process(const std::vector<ImageConstPtr> &msg_vec);
  void setupSync();
  void publishDebugImages(
      const std::vector<ImageConstPtr> &msg_vec,
      const std::vector<apriltag_ros::ApriltagVec> &detected_tags);
  bool guessCameraPose(const CamWorldPoints &wp, const CamImagePoints &ip,
                       CameraExtrinsics *cam0tf) const;
  // ---------- variables
  std::unique_ptr<MultiCamApriltagDetector> detector_;
  std::vector<std::shared_ptr<message_filters::Subscriber<ImageMsg>>> sub_;
  std::vector<image_transport::Publisher> imagePub_;
  std::unique_ptr<ApproxTimeSynchronizer2> approx_sync2_;
  std::unique_ptr<ApproxTimeSynchronizer3> approx_sync3_;
  std::unique_ptr<ExactSynchronizer2> sync2_;
  std::unique_ptr<ExactSynchronizer3> sync3_;
  image_transport::Subscriber singleCamSub_;
  std::vector<std::shared_ptr<TestCamera>> cameras_;
  bool useApproximateSync_{false};
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  bool useFileExtrinsics_{false};
  bool useFileIntrinsics_{false};
};
}  // namespace multicam_calibration
#endif
