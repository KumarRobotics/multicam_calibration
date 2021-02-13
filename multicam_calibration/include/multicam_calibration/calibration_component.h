/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#ifndef MULTICAM_CALIBRATION_CALIBRATION_COMPONENT_H
#define MULTICAM_CALIBRATION_CALIBRATION_COMPONENT_H

#include <multicam_calibration_srvs/srv/parameter_cmd.hpp>
#include "multicam_calibration/calibration_data.h"
#include "multicam_calibration/calibrator.h"
#include "multicam_calibration/multicam_apriltag_detector.h"

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <iostream>
#include <memory>

namespace multicam_calibration {

using ImageMsg = sensor_msgs::msg::Image;
using ImageConstPtr = ImageMsg::ConstSharedPtr;

typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>
    ApproxSyncPolicy2;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                        ImageMsg>
    ApproxSyncPolicy3;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                        ImageMsg, ImageMsg>
    ApproxSyncPolicy4;
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg>
    ApproxSyncPolicy5;
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg>
    ApproxSyncPolicy6;
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg>
    ApproxSyncPolicy7;
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg,
    ImageMsg>
    ApproxSyncPolicy8;
typedef message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg,
    ImageMsg, ImageMsg>
    ApproxSyncPolicy9;

typedef message_filters::Synchronizer<ApproxSyncPolicy2>
    ApproxTimeSynchronizer2;
typedef message_filters::Synchronizer<ApproxSyncPolicy3>
    ApproxTimeSynchronizer3;
typedef message_filters::Synchronizer<ApproxSyncPolicy4>
    ApproxTimeSynchronizer4;
typedef message_filters::Synchronizer<ApproxSyncPolicy5>
    ApproxTimeSynchronizer5;
typedef message_filters::Synchronizer<ApproxSyncPolicy6>
    ApproxTimeSynchronizer6;
typedef message_filters::Synchronizer<ApproxSyncPolicy7>
    ApproxTimeSynchronizer7;
typedef message_filters::Synchronizer<ApproxSyncPolicy8>
    ApproxTimeSynchronizer8;
typedef message_filters::Synchronizer<ApproxSyncPolicy9>
    ApproxTimeSynchronizer9;
typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg>
    ExactSynchronizer2;
typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg, ImageMsg>
    ExactSynchronizer3;

class CalibrationComponent : public rclcpp::Node {
 public:
  explicit CalibrationComponent(const rclcpp::NodeOptions &options);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void callback1(ImageConstPtr const &img0);
  void callback2(ImageConstPtr const &img0, ImageConstPtr const &img1);
  void callback3(ImageConstPtr const &img0, ImageConstPtr const &img1,
                 ImageConstPtr const &img2);
  void callback4(ImageConstPtr const &img0, ImageConstPtr const &img1,
                 ImageConstPtr const &img2, ImageConstPtr const &img3);
  void callback5(ImageConstPtr const &img0, ImageConstPtr const &img1,
                 ImageConstPtr const &img2, ImageConstPtr const &img3,
                 ImageConstPtr const &img4);
  void callback6(ImageConstPtr const &img0, ImageConstPtr const &img1,
                 ImageConstPtr const &img2, ImageConstPtr const &img3,
                 ImageConstPtr const &img4, ImageConstPtr const &img5);

  void initialize();
  bool calibrate(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool setParameter(
      const std::shared_ptr<
          multicam_calibration_srvs::srv::ParameterCmd::Request>
          req,
      std::shared_ptr<multicam_calibration_srvs::srv::ParameterCmd::Response>
          res);

  void process(const std::vector<ImageConstPtr> &msg_vec);
  void subscribe();
  void parseCameras(const std::string &fileName);
  void publishDebugImages(
      const std::vector<ImageConstPtr> &msg_vec,
      const std::vector<apriltag_ros::ApriltagVec> &detected_tags);
  void publishTagCounts();
  bool guessCameraPose(const CamWorldPoints &wp, const CamImagePoints &ip,
                       CameraExtrinsics *cam0tf, int frameNum) const;
  void writeCalibration(std::ostream &os, const CalibDataVec &results);
  std::string testCalibration(const CalibDataVec &calib);
  void updateCameras(const CalibDataVec &results);
  int getCameraIndex(const std::string &cam) const;
  cv::Mat flipRotate(const cv::Mat &img, int camIdx) const;

  bool readPointsFromFile(const std::string &fname);
  // ---------- variables
  std::unique_ptr<MultiCamApriltagDetector> detector_;
  std::vector<std::shared_ptr<message_filters::Subscriber<ImageMsg>>> sub_;
  std::vector<image_transport::Publisher> imagePub_;
  std::vector<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::UInt32>>>
      tagCountPub_;
  std::unique_ptr<ApproxTimeSynchronizer2> approx_sync2_;
  std::unique_ptr<ApproxTimeSynchronizer3> approx_sync3_;
  std::unique_ptr<ApproxTimeSynchronizer4> approx_sync4_;
  std::unique_ptr<ApproxTimeSynchronizer5> approx_sync5_;
  std::unique_ptr<ApproxTimeSynchronizer6> approx_sync6_;
  std::unique_ptr<ApproxTimeSynchronizer7> approx_sync7_;
  std::unique_ptr<ApproxTimeSynchronizer8> approx_sync8_;
  std::unique_ptr<ApproxTimeSynchronizer9> approx_sync9_;
  std::unique_ptr<ExactSynchronizer2> sync2_;
  std::unique_ptr<ExactSynchronizer3> sync3_;
  std::unique_ptr<Calibrator> calibrator_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrationService_;
  rclcpp::Service<multicam_calibration_srvs::srv::ParameterCmd>::SharedPtr
      parameterService_;
  image_transport::Subscriber singleCamSub_;
  std::vector<CamWorldPoints>
      worldPoints_;  // worldPoints_[camidx][frameidx][pointidx]
  std::vector<CamImagePoints> imagePoints_;
  CalibDataVec cameras_;
  CameraExtrinsics T_imu_body_;
  bool use_approximate_sync_{false};
  int frameNum_{0};
  int skipCount_{0};
  int skipFrames_{1};
  bool cameras_ready_;
  bool record_bag_;
  bool bagIsOpen_{false};
  std::string calibDir_;    // without the device name
  std::string deviceName_;  // name of camera rig, e.g. "stereo_cam"
  std::string bag_file_;
  // std::shared_ptr<rosbag2_cpp::Writer> bagWriter_;
};
}  // namespace multicam_calibration
#endif
