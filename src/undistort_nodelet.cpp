/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_calibration/undistort_nodelet.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>

namespace multicam_calibration {
  void UndistortNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    cameraInfoSub_ = nh.subscribe("camera_info_raw", 1,
                                  &UndistortNodelet::cameraInfoCallback, this);
    cameraInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    image_transport::ImageTransport it(nh);
    imageSub_ = it.subscribe("image", 1, &UndistortNodelet::imageCallback, this);
    imagePub_ = it.advertise("undist_image", 1);
    nh.param<double>("fov_scale", fovScale_, 1.0);
    nh.param<double>("balance", balance_, 0.0);
  }

  void UndistortNodelet::imageCallback(const ImageConstPtr &img) {
    if (mapx_.rows == 0) {
      ROS_WARN("no camera info received yet!");
      return;
    }
    if (imagePub_.getNumSubscribers() > 0) {
      bool isBayer = (img->encoding == "bayer_rggb8");
      bool outputColor = isBayer;
      std::string target_encoding = isBayer ?
        sensor_msgs::image_encodings::BGR8 :
        sensor_msgs::image_encodings::MONO8;
      cv::Mat im = cv_bridge::toCvCopy(img, target_encoding)->image;
      cv::Mat rectim;
      cv::remap(im, rectim, mapx_, mapy_, cv::INTER_LINEAR);
      std::string output_encoding = outputColor ? "bgr8" : "mono8";
      imagePub_.publish(cv_bridge::CvImage(img->header, output_encoding,
                                           rectim).toImageMsg());
    }
    undistortedCameraInfo_.header = img->header;
    if (cameraInfoPub_.getNumSubscribers() > 0) {
      cameraInfoPub_.publish(undistortedCameraInfo_);
    }
  }

  void UndistortNodelet::cameraInfoCallback(const CameraInfoConstPtr &camInfo) {
    if (camInfo->distortion_model != "equidistant" &&
        camInfo->distortion_model != "fisheye") {
      ROS_ERROR_STREAM("invalid distortion model in camera_info: "
                       << camInfo->distortion_model);
      cameraInfoSub_.shutdown();
      return;
    }
    undistortedCameraInfo_ = *camInfo;
    undistortedCameraInfo_.distortion_model = "plumb_bob";
    cv::Mat mapx, mapy;
    const cv::Size old_sz(camInfo->width, camInfo->height);
    const cv::Size new_sz(old_sz);
    const cv::Mat K(3, 3, CV_64F, (void *)&camInfo->K[0]);
    const cv::Mat D(4, 1, CV_64F, (void *)&camInfo->D[0]);
    const cv::Mat R = cv::Mat::eye(3,3, CV_64F);
    // let opencv compute the optimal camera matrix
    cv::Mat P;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, D, old_sz, R, P,
                                                            balance_, new_sz,
                                                            fovScale_);
    cv::fisheye::initUndistortRectifyMap(K, D, R, P, new_sz,
                                         CV_32FC1, mapx_, mapy_);
    ROS_INFO("got camera_info callback, initializing undistort map!");
    ROS_INFO_STREAM("fov scale: " << fovScale_ << " balance: " << balance_);
    ROS_INFO_STREAM("original K matrix: " << std::endl << K);
    ROS_INFO_STREAM("distortion coefficients: " << std::endl << D.t());
    ROS_INFO_STREAM("P matrix: " << std::endl << P);
    undistortedCameraInfo_.D.clear();  // no distortion!
    undistortedCameraInfo_.R = {1.0, 0.0, 0.0,   0.0, 1.0, 0.0,   0.0, 0.0, 1.0};
    const double *cK = &P.at<double>(0,0);
    undistortedCameraInfo_.K = {cK[0],  cK[1],  cK[2],
                                cK[3],  cK[4],  cK[5],
                                cK[6],  cK[7],  cK[8]};
    undistortedCameraInfo_.P = {cK[0],  cK[1],  cK[2], 0.0,
                                cK[3],  cK[4],  cK[5], 0.0,
                                cK[6],  cK[7],  cK[8], 0.0};
    cameraInfoSub_.shutdown();
  }
}
