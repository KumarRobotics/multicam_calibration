/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#ifndef MULTICAM_CALIBRATION_MULTICAM_APRILTAG_DETECTOR_H
#define MULTICAM_CALIBRATION_MULTICAM_APRILTAG_DETECTOR_H

#include <apriltag_ros/apriltag_detector.h>
#include <multicam_calibration/types.h>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace multicam_calibration {
using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
class MultiCamApriltagDetector {
 public:
  MultiCamApriltagDetector(rclcpp::Node *node, const std::string &tagFamily,
                           int detectorType, int blackBorderSize, int tagRows,
                           int tagCols, float tagSize, float tagSpacing,
                           const std::string &outfname);

  std::vector<apriltag_ros::ApriltagVec> process(
      std::vector<ImageConstPtr> const &msg_vec, CamWorldPoints *wp,
      CamImagePoints *ip);

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  struct Point2f {
    float x;
    float y;
  };
  struct tag_corners_t {
    bool found = false;
    std::array<Point2f, 4> corners;
  };

  void image_cb(const ImageConstPtr &msg);
  std::array<Point2f, 4> get_tag_corner_positions(unsigned int tag_id) const;

  int target_tag_cols_, target_tag_rows_;
  float target_tag_size_, target_tag_spacing_ratio_;
  apriltag_ros::ApriltagDetector::Ptr detector_;
  std::ofstream outfile_;
  unsigned int frame_count_{0};
  std::vector<int> tag_count_;
  rclcpp::Node *node_;
};
}  // namespace multicam_calibration

#endif
