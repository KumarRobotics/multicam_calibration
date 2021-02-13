/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#include "multicam_calibration/camera_extrinsics.h"
#include <boost/range/irange.hpp>

namespace multicam_calibration {

  CameraExtrinsics zeros() {
    return (CameraExtrinsics::Zero());
  }

  CameraExtrinsics identity() {
    return (CameraExtrinsics::Identity());
  }
  bool isNonZero(const CameraExtrinsics &T) {
    return ((T(0,0) + T(0,1) + T(0,2) + T(0,3) +
             T(1,0) + T(1,1) + T(1,2) + T(1,3) +
             T(2,0) + T(2,1) + T(2,2) + T(2,3) +
             T(3,0) + T(3,1) + T(3,2) + T(3,3)) != 0);
  }
}
