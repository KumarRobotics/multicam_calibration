/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_calibration/camera_intrinsics.h"

namespace multicam_calibration {
  std::ostream &operator<<(std::ostream &os, const CameraIntrinsics &ci) {
    os << "camera mod: " << ci.camera_model << std::endl;
    os << "intrinsics: " << ci.intrinsics[0] << " " << ci.intrinsics[1]
       << " " << ci.intrinsics[2] << " " << ci.intrinsics[3] << std::endl;
    os << "distortion model: " << ci.distortion_model << std::endl;
    os << "distortion coeffs: ";
    for (const auto &d: ci.distortion_coeffs) {
      os << " " << d;
    }
    os << std::endl;
    os << "resolution:" << ci.resolution[0] << " x " << ci.resolution[1];
    return (os);
  }
}
