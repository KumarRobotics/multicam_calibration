/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_CALIBRATION_TESTRECTIFY_H
#define MULTICAM_CALIBRATION_TESTRECTIFY_H

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <apriltag_ros/apriltag_detector.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <memory>

namespace multicam_calibration {
  using sensor_msgs::Image;
  using sensor_msgs::ImageConstPtr;
  using sensor_msgs::CameraInfo;
  using sensor_msgs::CameraInfoConstPtr;
  using message_filters::TimeSynchronizer;
  using message_filters::Subscriber;
  
  class TestRectify {
  public:
    TestRectify(const ros::NodeHandle &pnh);
    ~TestRectify();

    TestRectify(const TestRectify&) = delete;
    TestRectify& operator=(const TestRectify&) = delete;
    
    void imageCallback(const ImageConstPtr &msg0, const ImageConstPtr &msg1);
    bool initialize();
  private:
    void publishImage(const sensor_msgs::ImageConstPtr& origMsg,
                      const cv::Mat &img);
    ros::NodeHandle           nh_;
    std::vector<std::string>  topic_;
    apriltag_ros::ApriltagDetector::Ptr detector_;
    image_transport::ImageTransport	imTrans_;
    image_transport::Publisher		  imPub_;
    std::shared_ptr<Subscriber<Image> > imageSub_[2];
    std::shared_ptr<TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> > sync_;
  };

}

#endif
