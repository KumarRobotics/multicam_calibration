/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#ifndef MULTICAM_CALIBRATION_CAMERA_INTRINSICS_H
#define MULTICAM_CALIBRATION_CAMERA_INTRINSICS_H

#include <vector>
#include <string>
#include <iostream>

namespace multicam_calibration {
  struct CameraIntrinsics {
    CameraIntrinsics() {
      intrinsics.resize(4);
      distortion_coeffs.resize(4);
      resolution.resize(2);
    }
    std::vector<double> distortion_coeffs;
    std::vector<double> intrinsics; // K Matrix
    std::vector<int>    resolution;
    std::string distortion_model;
    std::string camera_model;
  };
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics &ci);
}
#endif
