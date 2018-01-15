/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#include <multicam_calibration/frame_residual.h>
#include <multicam_calibration/calibrator.h>
#include <multicam_calibration/utils.h>
#include <ceres/ceres.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/range/irange.hpp>
#include <fstream>
#include <iomanip>

#define CERES_PUBLIC_INTERNAL_CONFIG_H_
#include <ceres/ceres.h>

//#define DEBUG_PARAMS

namespace multicam_calibration {
  using utils::Mat;
  using utils::Vec;
  using utils::DynVec;
  using utils::rotation_matrix;
  using boost::irange;
  
  void Calibrator::addCamera(const std::string &name,
                             const CalibrationData &calibData) {
    calibrationData_.push_back(calibData);
    worldPoints_.resize(calibrationData_.size());
    imagePoints_.resize(calibrationData_.size());
  }

  void Calibrator::addPoints(int frameNum, const CamWorldPoints &wp,
                             const CamImagePoints &ip,
                             const CameraExtrinsics &camPoseGuess) {
    if (wp.size() != calibrationData_.size() ||
        ip.size() != calibrationData_.size()) {
      std::cerr << "number of cams != width of array!" << std::endl;
      return;
    }
    for (unsigned int cam_idx  = 0; cam_idx < calibrationData_.size(); cam_idx++) {
      FrameWorldPoints fwp(wp[cam_idx].begin(), wp[cam_idx].end());
      worldPoints_[cam_idx].push_back(fwp);
      FrameImagePoints fip(ip[cam_idx].begin(), ip[cam_idx].end());
      imagePoints_[cam_idx].push_back(fip);
    }
    rigPoseGuess_.push_back(camPoseGuess);
  }

  void Calibrator::initializeVariables(std::vector<double> *param_ptr) {
    std::vector<double> &params = *param_ptr;
    const unsigned int num_cameras = calibrationData_.size();
    
    // initialize intrinsics
    for (const auto cam_idx : irange(0u, num_cameras)) {
      const CameraIntrinsics &ci = calibrationData_[cam_idx].intrinsics;
      params.insert(params.end(), std::begin(ci.intrinsics),
                    std::end(ci.intrinsics));
      params.insert(params.end(), std::begin(ci.distortion_coeffs),
                    std::end(ci.distortion_coeffs));
    }
    // initialize T_cn_cnm1, only for cameras > 0
    for(const auto cam_idx : irange(1u, num_cameras)) {
      // ------------ relative to camera 0 -----------------
      std::vector<double> rvec_tvec = utils::transform_to_rvec_tvec(calibrationData_[cam_idx].T_cn_cnm1);
      params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
    }
    // the number of frames are the same across all cameras,
    // although not all frames must contain detected points
    const unsigned int num_frames = worldPoints_[0].size();
    // initialize pose guess for camera 0
    for(const auto i : irange(0u, num_frames))  {
      // add cam pose guess for cameras 0-(n-2) for each frame
      std::vector<double> rvec_tvec = utils::transform_to_rvec_tvec(rigPoseGuess_[i]);
      params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
    };
#ifdef DEBUG_PARAMS    
    utils::print_params(params, calibrationData_);
#endif
  }

  void Calibrator::runCalibration() {
    if (calibrationData_.empty()) {
      std::cerr << "no data to run on!" << std::endl;
      return;
    }
    params_.clear();
    initializeVariables(&params_);
    ceres::Problem problem;
    setupOptimizationProblem(&problem, &params_);
    std::cout << "Num params: " << params_.size() << std::endl;
    std::cout << "Num residuals: " << problem.NumResiduals() << std::endl;

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;
    options.num_threads = 4;
    options.function_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

#ifdef DEBUG_PARAMS
    utils::print_params(params_, calibrationData_);
#endif
    std::cout << summary.FullReport() << std::endl;
  }

  void Calibrator::testCalibration(Residuals *resptr) {
    auto &res = *resptr; // residuals
    const unsigned int num_frames = worldPoints_[0].size();
    const unsigned int num_cameras = calibrationData_.size();

    unsigned int extrinsicsBase = utils::get_extrinsics_base(calibrationData_);
    // add the extrinsics between cam0 and the rest
    unsigned int totNumCameraParams = extrinsicsBase + 6 * (num_cameras - 1);
    
    for (const auto fnum : irange(0u, num_frames))  {
      std::vector<FrameWorldPoints> frame_world_points;
      std::vector<FrameImagePoints> frame_image_points;
      unsigned int frame_num_points = 0;
      for (const auto cam_idx : irange(0u, num_cameras)) {
        frame_world_points.push_back(worldPoints_[cam_idx][fnum]);
        frame_image_points.push_back(imagePoints_[cam_idx][fnum]);
        frame_num_points += frame_world_points[cam_idx].size();
      }
      const auto fr = FrameResidual(frame_world_points, frame_image_points,
                                    calibrationData_, extrinsicsBase);
      const unsigned numResiduals = 2 * frame_num_points;
      std::vector<double> residuals(numResiduals);
      // point offset to cam0 pose for this particular frame
      const auto R_vec_offset = totNumCameraParams + 6 * fnum;
      double const * const params[2] = {&params_[0], &params_[R_vec_offset]};
      // compute residuals
      fr(params, residuals.data());
      // unpack residuals into vector of vectors of vectors
      res.push_back(std::vector<std::vector<Vec2d>>()); // empty for this frame
      for (const auto cam_idx : irange(0u, num_cameras)) {
        res.back().push_back(std::vector<Vec2d>()); // empty for this camera
        for (const auto res_idx : irange(0u, (unsigned int)worldPoints_[cam_idx][fnum].size())) {
          res.back().back().push_back(Vec2d(residuals[res_idx * 2], residuals[res_idx * 2 + 1]));
        }
      }
    }
  }

  CalibDataVec Calibrator::getCalibrationResults() const {
    CalibDataVec res;
    if (calibrationData_.empty()) {
      return (res);
    }
    const CameraExtrinsics &T_cam0_imu = calibrationData_[0].T_cam_imu;
    const std::vector<double> &p = params_;
    
    CameraExtrinsics T_cn_cam0 = identity();
    unsigned int extrinsicsBase = utils::get_extrinsics_base(calibrationData_);
    unsigned int intrinsics_offset = 0;

    for (unsigned int cam_idx = 0; cam_idx < calibrationData_.size(); cam_idx++) {
      CalibrationData cd(calibrationData_[cam_idx]);
      if (cam_idx > 0) {
        unsigned int off = extrinsicsBase + 6 * (cam_idx - 1);
        const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
        Mat<double, 3, 3>       R = rotation_matrix(rvec);
        const Vec<double, 3>    t = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);
        CameraExtrinsics T_cn_cnm1 = zeros();

        T_cn_cnm1.block<3,3>(0, 0) = R;
        T_cn_cnm1.block<3,1>(0, 3) = t;
        T_cn_cnm1(3,3) = 1.0;
        
        cd.T_cn_cnm1 = T_cn_cnm1;
        // propagate forward to T_cn_cam0
        T_cn_cam0 = cd.T_cn_cnm1 * T_cn_cam0;
        if (isNonZero(T_cam0_imu)) {
          cd.T_cam_imu = T_cn_cam0 * T_cam0_imu;
        }
      }
      // offset for intrinsics
      for (unsigned int i = 0; i < cd.intrinsics.intrinsics.size(); i++) {
        cd.intrinsics.intrinsics[i] = p[intrinsics_offset + i];
      }
      for (unsigned int i = 0; i < cd.intrinsics.distortion_coeffs.size(); i++) {
        cd.intrinsics.distortion_coeffs[i] = p[intrinsics_offset +
                                               cd.intrinsics.intrinsics.size() + i];
      }
      res.push_back(cd);
      intrinsics_offset += cd.intrinsics.intrinsics.size() + cd.intrinsics.distortion_coeffs.size();
    }
    return (res);
  }


  void Calibrator::setupOptimizationProblem(ceres::Problem *prob, std::vector<double> *vars) {
    std::vector<double> &params = *vars;
    ceres::Problem     &problem = *prob;
    
    const unsigned int num_frames = worldPoints_[0].size();
    const unsigned int num_cameras = calibrationData_.size();

    unsigned int extrinsicsBase = utils::get_extrinsics_base(calibrationData_);
    // add the extrinsics between cam0 and the rest
    unsigned int totNumCameraParams = extrinsicsBase + 6 * (num_cameras - 1);

    //
    // Create one FrameResidual for each frame
    //
    
    for(const auto i : irange(0u, num_frames))  {
      std::vector<FrameWorldPoints> frame_world_points;
      std::vector<FrameImagePoints> frame_image_points;
      unsigned int frame_num_points = 0;
      for(const auto cam_idx : irange(0u, num_cameras)) {
        frame_world_points.push_back(worldPoints_[cam_idx][i]);
        frame_image_points.push_back(imagePoints_[cam_idx][i]);
        frame_num_points += frame_world_points[cam_idx].size();
      }
      auto cost_function = new ceres::DynamicAutoDiffCostFunction<FrameResidual>(
        new FrameResidual(frame_world_points, frame_image_points, calibrationData_,
                          extrinsicsBase));
      // Per camera intrinsics + (ncam-1) extrinsics
      cost_function->AddParameterBlock(totNumCameraParams);
      // Pose of rig wrt calib board
      cost_function->AddParameterBlock(6);
       // Reprojection error
      cost_function->SetNumResiduals(2 * frame_num_points);
      // point offset to cam0 pose for this particular frame
      const auto R_vec_offset = totNumCameraParams + 6 * i;
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                               &params[0], &params[R_vec_offset]);
      //problem.SetParameterBlockConstant(&params[0]);
    }
  }
}
