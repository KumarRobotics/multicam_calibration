/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_CALIBRATION_IMAGE_CONVERSION_NODELET_H
#define MULTICAM_CALIBRATION_IMAGE_CONVERSION_NODELET_H

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>


namespace multicam_calibration {
  using sensor_msgs::ImageConstPtr;
  class ImageConversionNodelet : public nodelet::Nodelet {
  public:
    void onInit() override;
  private:
    void imageCallback(const ImageConstPtr &img);
    // ---------- variables

    image_transport::Subscriber imageSub_;
    image_transport::Publisher  imagePub_;
  };
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multicam_calibration::ImageConversionNodelet, nodelet::Nodelet)

#endif
