/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#ifndef MULTICAM_CALIBRATION_CAMERA_EXTRINSICS_H
#define MULTICAM_CALIBRATION_CAMERA_EXTRINSICS_H
#include <Eigen/Core>

namespace multicam_calibration {
  using  CameraExtrinsics = Eigen::Matrix<double, 4, 4>;
  CameraExtrinsics zeros();
  CameraExtrinsics identity();
  bool isNonZero(const CameraExtrinsics &T);
  typedef std::vector<CameraExtrinsics, Eigen::aligned_allocator<CameraExtrinsics> > CameraExtrinsicsVec;
}
#endif
