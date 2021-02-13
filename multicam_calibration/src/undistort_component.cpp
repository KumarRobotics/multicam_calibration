/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_calibration/undistort_component.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace multicam_calibration {
UndistortComponent::UndistortComponent(const rclcpp::NodeOptions &options)
    : Node("undistort", options) {
  const auto qos = rmw_qos_profile_default;

  cameraInfoSub_ = create_subscription<CameraInfo>(
      "camera_info_raw", 1,
      std::bind(&UndistortComponent::cameraInfoCallback, this,
                std::placeholders::_1));
  cameraInfoPub_ = create_publisher<CameraInfo>("camera_info", 1);
  imageSub_ = image_transport::create_subscription(
      this, "image",
      std::bind(&UndistortComponent::imageCallback, this,
                std::placeholders::_1),
      "raw", qos);
  imagePub_ = image_transport::create_publisher(this, "undist_image", qos);

  fovScale_ = declare_parameter("fov_scale", 1.0);
  balance_ = declare_parameter("balance", 1.0);
  ignoreWrongDistortion_ = declare_parameter("ignore_wrong_distortion", false);
}

void UndistortComponent::imageCallback(const ImageConstPtr &img) {
  if (mapx_.rows == 0) {
    RCLCPP_WARN(get_logger(), "no camera info received yet!");
    return;
  }
  if (imagePub_.getNumSubscribers() > 0) {
    bool isBayer = (img->encoding == "bayer_rggb8");
    bool outputColor = isBayer;
    std::string target_encoding = isBayer ? sensor_msgs::image_encodings::BGR8
                                          : sensor_msgs::image_encodings::MONO8;
    cv::Mat im = cv_bridge::toCvCopy(img, target_encoding)->image;
    cv::Mat rectim;
    cv::remap(im, rectim, mapx_, mapy_, cv::INTER_LINEAR);
    std::string output_encoding = outputColor ? "bgr8" : "mono8";
    imagePub_.publish(
        cv_bridge::CvImage(img->header, output_encoding, rectim).toImageMsg());
  }
  undistortedCameraInfo_.header = img->header;
  if (cameraInfoPub_->get_subscription_count() > 0) {
    cameraInfoPub_->publish(undistortedCameraInfo_);
  }
}

void UndistortComponent::cameraInfoCallback(CameraInfoConstPtr camInfo) {
  if (camInfo->distortion_model != "equidistant" &&
      camInfo->distortion_model != "fisheye" && !ignoreWrongDistortion_) {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "invalid distortion model in camera_info: "
                            << camInfo->distortion_model);
    cameraInfoSub_.reset();
    return;
  }
  undistortedCameraInfo_ = *camInfo;
  undistortedCameraInfo_.distortion_model = "plumb_bob";
  cv::Mat mapx, mapy;
  const cv::Size old_sz(camInfo->width, camInfo->height);
  const cv::Size new_sz(old_sz);
  const cv::Mat K(3, 3, CV_64F, (void *)&camInfo->k[0]);
  const cv::Mat D(4, 1, CV_64F, (void *)&camInfo->d[0]);
  const cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  // let opencv compute the optimal camera matrix
  cv::Mat P;
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K, D, old_sz, R, P, balance_, new_sz, fovScale_);
  cv::fisheye::initUndistortRectifyMap(K, D, R, P, new_sz, CV_32FC1, mapx_,
                                       mapy_);
  RCLCPP_INFO(get_logger(),
              "got camera_info callback, initializing undistort map!");
  RCLCPP_INFO_STREAM(get_logger(),
                     "fov scale: " << fovScale_ << " balance: " << balance_);
  RCLCPP_INFO_STREAM(get_logger(), "original K matrix: " << std::endl << K);
  RCLCPP_INFO_STREAM(get_logger(), "distortion coefficients: " << std::endl
                                                               << D.t());
  RCLCPP_INFO_STREAM(get_logger(), "P matrix: " << std::endl << P);
  undistortedCameraInfo_.d.clear();  // no distortion!
  undistortedCameraInfo_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  const double *cK = &P.at<double>(0, 0);
  undistortedCameraInfo_.k = {cK[0], cK[1], cK[2], cK[3], cK[4],
                              cK[5], cK[6], cK[7], cK[8]};
  undistortedCameraInfo_.p = {cK[0], cK[1], cK[2], 0.0,   cK[3], cK[4],
                              cK[5], 0.0,   cK[6], cK[7], cK[8], 0.0};
  cameraInfoSub_.reset();
}
}  // namespace multicam_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_calibration::UndistortComponent)
