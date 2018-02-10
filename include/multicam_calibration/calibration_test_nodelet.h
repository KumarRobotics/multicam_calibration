/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohtha
 */

#ifndef MULTICAM_CALIBRATION_CALIBRATION_TEST_NODELET_H
#define MULTICAM_CALIBRATION_CALIBRATION_TEST_NODELET_H

#include "multicam_calibration/multicam_apriltag_detector.h"
#include "multicam_calibration/calibration_data.h"

#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>


namespace multicam_calibration {
  using sensor_msgs::ImageConstPtr;
  using sensor_msgs::CameraInfo;
  using sensor_msgs::CameraInfoConstPtr;
  using ImageMsg = sensor_msgs::Image;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> ApproxSyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg> ApproxSyncPolicy3;
  typedef message_filters::Synchronizer<ApproxSyncPolicy2> ApproxTimeSynchronizer2;
  typedef message_filters::Synchronizer<ApproxSyncPolicy3> ApproxTimeSynchronizer3;
  typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg> ExactSynchronizer2;
  typedef message_filters::TimeSynchronizer<ImageMsg, ImageMsg, ImageMsg> ExactSynchronizer3;
  class CalibrationTestNodelet : public nodelet::Nodelet {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void onInit() override;
    struct TestCamera : public CalibrationData {
      TestCamera(const CalibrationData &cd) : CalibrationData(cd) {
      };
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      void camInfoCallback(const CameraInfoConstPtr &caminfo);
      ros::Subscriber camInfoSub_;
      bool useCameraInfo_{true};
    };

  private:
    void callback1(ImageConstPtr const &img0);
    void callback2(ImageConstPtr const &img0, ImageConstPtr const &img1);
    void callback3(ImageConstPtr const &img0, ImageConstPtr const &img1, ImageConstPtr const &img2);
    void process(const std::vector<ImageConstPtr> &msg_vec);
    void setupSync();
    void publishDebugImages(const std::vector<ImageConstPtr> &msg_vec,
                            const std::vector<apriltag_ros::ApriltagVec> &detected_tags);
    bool guessCameraPose(const CamWorldPoints &wp, const CamImagePoints &ip, CameraExtrinsics *cam0tf) const;
    // ---------- variables
    std::unique_ptr<MultiCamApriltagDetector> detector_;
    std::vector<std::shared_ptr<image_transport::SubscriberFilter> > sub_;
    std::vector<ros::Publisher> imagePub_;
    std::unique_ptr<ApproxTimeSynchronizer2> approx_sync2_;
    std::unique_ptr<ApproxTimeSynchronizer3> approx_sync3_;
    std::unique_ptr<ExactSynchronizer2> sync2_;
    std::unique_ptr<ExactSynchronizer3> sync3_;
    ros::Subscriber singleCamSub_;
    std::vector<std::shared_ptr<TestCamera>> cameras_;
    bool useApproximateSync_{false};
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
    tf2_ros::Buffer tfBuffer_;
    bool useFileExtrinsics_{false};
    bool useFileIntrinsics_{false};
  };
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multicam_calibration::CalibrationTestNodelet, nodelet::Nodelet)

#endif
