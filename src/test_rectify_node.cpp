/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include "multicam_calibration/test_rectify.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_rectify");
  ros::NodeHandle pnh("~");

  try {
    multicam_calibration::TestRectify testRect(pnh);
    testRect.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
