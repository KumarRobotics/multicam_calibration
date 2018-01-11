/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohtha
 */

#ifndef MULTICAM_CALIBRATION_CALIBRATION_DATA_H
#define MULTICAM_CALIBRATION_CALIBRATION_DATA_H

#include "multicam_calibration/camera_intrinsics.h"
#include "multicam_calibration/camera_extrinsics.h"
#include "multicam_calibration/types.h"
#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <ostream>
#include <ros/ros.h>


namespace multicam_calibration {
  struct CalibrationData {
    typedef std::vector<CalibrationData, Eigen::aligned_allocator<CalibrationData> > CalibDataVec;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string       name;
    CameraIntrinsics  intrinsics;
    CameraExtrinsics  T_cam_imu;
    CameraExtrinsics  T_cn_cnm1;
    std::string       rostopic;
    int tagCount{0};
    // 
    static CalibDataVec parse_cameras(const ros::NodeHandle &nh);
  };
  using CalibDataVec=CalibrationData::CalibDataVec;
  std::ostream &operator<<(std::ostream &os, const CalibrationData &cd);
}
#endif
