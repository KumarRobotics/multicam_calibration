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
  }

  void UndistortNodelet::imageCallback(const ImageConstPtr &img) {
    if (mapx_.rows == 0) {
      ROS_WARN("no camera info received yet!");
      return;
    }
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
    cameraInfo_.header = img->header;
    cameraInfoPub_.publish(cameraInfo_);
  }

  void UndistortNodelet::cameraInfoCallback(const CameraInfoConstPtr &camInfo) {
    cameraInfo_ = *camInfo;
    cameraInfo_.distortion_model = "plumb_bob";
    cv::Mat mapx, mapy;
    cv::Size new_sz(camInfo->width, camInfo->height);
    cv::Mat K(3, 3, CV_64F, (void *)&camInfo->K[0]);
    cv::Mat D(4, 1, CV_64F, (void *)&camInfo->D[0]);
    cv::Mat R = cv::Mat::eye(3,3, CV_64F);
    cv::fisheye::initUndistortRectifyMap(K, D, R, K, new_sz,
                                         CV_32FC1, mapx_, mapy_);
    cameraInfo_.D.clear();  // no distortion!
    cameraInfo_.R = {1.0, 0.0, 0.0,   0.0, 1.0, 0.0,   0.0, 0.0, 1.0};
    const double *cK = &camInfo->K[0];
    cameraInfo_.P = {cK[0],  cK[1],  cK[2], 0.0,
                     cK[3],  cK[4],  cK[5], 0.0,
                     cK[6],  cK[7],  cK[8], 0.0};
    cameraInfoSub_.shutdown();
  }
}
