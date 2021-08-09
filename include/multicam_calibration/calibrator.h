/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#ifndef MULTICAM_CALIBRATION_CALIBRATOR_H
#define MULTICAM_CALIBRATION_CALIBRATOR_H

#include "multicam_calibration/calibration_data.h"
#include <vector>

namespace ceres {
  class Problem;
}

namespace multicam_calibration {
  class Calibrator {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Residuals = std::vector<std::vector<std::vector<Vec2d>>>;
    void setFixAllIntrinsics(bool f) { fixAllIntrinsics_ = f; }
    void showCameraStatus() const;
    void setCameras(const CalibDataVec &calibData);
    void addPoints(int frameNum, const CamWorldPoints &wp, const CamImagePoints &ip,
                   const CameraExtrinsics &cam0InitPose);
    void runCalibration();
    CalibDataVec getCalibrationResults() const;
    // returns residuals[frame_num][cam_idx][point_idx]
    void testCalibration(Residuals *resptr);
    // removes outliers with reprojection error more than maxErr pixels
    void removeOutliers(double maxErr);

  private:
    void initializeVariables(std::vector<double> *param_ptr);
    void setupOptimizationProblem(ceres::Problem *prob, std::vector<double> *vars);
    size_t getPointsAndMasks(size_t fnum,
                             std::vector<FrameWorldPoints> *frame_world_points,
                             std::vector<FrameImagePoints> *frame_image_points,
                             std::vector<std::vector<bool>> *masks) const;

    CalibDataVec                  calibrationData_;
    std::vector<CamWorldPoints>   worldPoints_;
    std::vector<CamImagePoints>   imagePoints_;
    std::vector<std::vector<std::vector<bool>>> masks_;
    CameraExtrinsicsVec           cam0PoseGuess_; // initial pose guess cam0, one per frame
    std::vector<double>           params_; // all the parameters that need to be optimized
    bool                          fixAllIntrinsics_{false};
  };
}
#endif
