/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_calibration/image_conversion_nodelet.h"
#include <cv_bridge/cv_bridge.h>

namespace multicam_calibration {
  static  cv::Mat scale_to_255(const cv::Mat &a) {
    cv::Point minLoc, maxLoc;
    double minVal, maxVal;
    cv::minMaxLoc(a, &minVal, &maxVal, &minLoc, &maxLoc);
    double range = maxVal - minVal;
    cv::Mat scaled;
    cv::convertScaleAbs(a, scaled, 255.0/range, -minVal*255.0/range);
    return (scaled);
  }

  void ImageConversionNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    image_transport::ImageTransport it(nh);
    imageSub_ = it.subscribe("image", 1, &ImageConversionNodelet::imageCallback, this);
    imagePub_ = it.advertise("converted_image", 1);
  }

  void ImageConversionNodelet::imageCallback(const ImageConstPtr &img) {
    if (imagePub_.getNumSubscribers() > 0) {
      cv::Mat im;
      if (img->encoding == "mono16") {
        cv::Mat im16 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16)->image;
        im = scale_to_255(im16);
      } else {
        im = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8)->image;
      }
      imagePub_.publish(cv_bridge::CvImage(img->header, "mono8",
                                           im).toImageMsg());
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multicam_calibration::ImageConversionNodelet, nodelet::Nodelet)
