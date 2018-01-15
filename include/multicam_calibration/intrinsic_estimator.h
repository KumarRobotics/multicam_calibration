/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 *      Jason Owens jlowens+upenn@gmail.com
 */

#pragma once

#include "multicam_calibration/calibration_data.h"
#include <vector>

namespace ros {
  class NodeHandle;
}

namespace multicam_calibration {
  bool estimate_intrinsics(ros::NodeHandle& nh, size_t num_images, CalibrationData& out_data);
}
