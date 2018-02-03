/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibrator.h"
#include "multicam_calibration/utils.h"
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

  // find number of intrinsics for all cameras
  static unsigned int get_num_intrinsics(const CalibDataVec &cdv) {
    unsigned int extrinsicsBase(0);
    for (const auto &cd : cdv) {
      // for each camera we have fx, fy, cx, cy + distortion coefficients;
      extrinsicsBase += cd.intrinsics.intrinsics.size() + cd.intrinsics.distortion_coeffs.size();
    }
    return (extrinsicsBase);
  }

  // maps parameters to parameter blocks
  // ------ parameter block layout --------
  //
  // params[0]      = intrinsics for all cameras
  // params[1]      = extrinsics cam1->cam0 (if cam1 exists)
  //     ...
  // params[ncam-1] = extrinsics camn->cam0
  // params[ncam]   = frame_tf

  static std::vector<double *>
  params_to_blocks(const CalibDataVec &cdv, std::vector<double> &params,
                   unsigned int frame_num) {
    std::vector<double *> v;
    unsigned int num_cameras = cdv.size();
       v.push_back(&params[0]); // intrinsics are the first block
    // number of intrinsic parameters for all cameras
    unsigned int numIntrinsics = get_num_intrinsics(cdv);
    // add the extrinsics between cam0 and the rest
    unsigned int totNumCameraParams = numIntrinsics + 6 * (num_cameras - 1);
    for (const auto iext : irange(0u, num_cameras - 1)) {
      v.push_back(&params[numIntrinsics + 6 * iext]);
    }
    const auto R_vec_offset = totNumCameraParams + 6 * frame_num;
    v.push_back(&params[R_vec_offset]);
    return (v);
  }


  struct FrameResidual
  {
    // Input is the list of world points and image points for the current frame in
    // each of the cameras
    FrameResidual(const std::vector<FrameWorldPoints> &world_points,
                  const std::vector<FrameImagePoints> &image_points,
                  const CalibDataVec &cams) :
      world_points_(world_points), image_points_(image_points), cams_(cams)
      {
      }

    template <typename T>
    bool operator()(T const *const *params, T *residual) const
      {
        unsigned int frame_tf_idx   = cams_.size();
        const Vec<T, 3> R_vec_frame = Eigen::Map<const Vec<T, 3>>(&params[frame_tf_idx][0]);
        const Mat<T, 3, 3>  R_frame = rotation_matrix(R_vec_frame);
        const Vec<T, 3>     t_frame = Eigen::Map<const Vec<T, 3>>(&params[frame_tf_idx][3]);

        unsigned int residual_count = 0;
        unsigned int intrinsics_offset = 0;
        for(const auto cam_idx : irange(0u, (unsigned int)cams_.size()))
          {
            const CalibrationData &cam = cams_[cam_idx];
            Mat<T, 3, 3> K = Mat<T, 3, 3>::Identity();
            K(0, 0) = params[0][intrinsics_offset];
            K(1, 1) = params[0][intrinsics_offset + 1];
            K(0, 2) = params[0][intrinsics_offset + 2];
            K(1, 2) = params[0][intrinsics_offset + 3];
            unsigned int numDist = cam.intrinsics.distortion_coeffs.size();
            DynVec<T> D(numDist);
            for(const auto i : irange(0u, numDist)) {
              D(i) = params[0][intrinsics_offset +
                               cam.intrinsics.intrinsics.size() + i];
            }

            Mat<T, 3, 3> R_cam;
            Vec<T, 3> t_cam;
            if(cam_idx == 0)
              {
                R_cam = Mat<T, 3, 3>::Identity();
                t_cam = Vec<T, 3>::Zero();
              }
            else
              {
                // apply camera extrinsics wrt camera 0
                const Vec<T, 3> R_vec_cam =
                  Eigen::Map<const Vec<T, 3>>(&params[cam_idx][0]);
                R_cam = rotation_matrix(R_vec_cam); // rotation vector
                t_cam = Eigen::Map<const Vec<T, 3>>(&params[cam_idx][3]);
                std::cout.flush();
              }

            const Mat<T, 3, 3> R = R_cam * R_frame;
            const Vec<T, 3> t = R_cam * t_frame + t_cam;
            vector_aligned<Point2<T>> projected_points;
            if (cam.intrinsics.distortion_model == "equidistant") {
              projected_points = utils::project_frame_equidistant(world_points_[cam_idx], R, t, K, D);
            } else if (cam.intrinsics.distortion_model == "radtan") {
              projected_points = utils::project_frame_radtan(world_points_[cam_idx], R, t, K, D);
            } else {
              std::cout << "ERROR: unknown distortion model: " << cam.intrinsics.distortion_model << std::endl;
              return (false);
            }
            for(const auto i : irange<size_t>(0, projected_points.size()))
              {
                residual[residual_count++] =
                  projected_points[i].x - T(image_points_[cam_idx][i].x);
                residual[residual_count++] =
                  projected_points[i].y - T(image_points_[cam_idx][i].y);
              }
            intrinsics_offset += cam.intrinsics.intrinsics.size() +
              cam.intrinsics.distortion_coeffs.size();
          }
        return true;
      }

  private:
    const std::vector<FrameWorldPoints> world_points_;
    const std::vector<FrameImagePoints> image_points_;
    CalibDataVec cams_;
  };

  static std::vector<double> transform_to_rvec_tvec(const CameraExtrinsics &tf) {
    std::vector<double> v;
    Eigen::AngleAxisd axisAngle(tf.block<3,3>(0, 0));
    Eigen::Vector3d rvec = axisAngle.axis() * axisAngle.angle();
    v.push_back(rvec(0));
    v.push_back(rvec(1));
    v.push_back(rvec(2));
    // Translation
    Eigen::Vector3d T = tf.block<3,1>(0, 3);
    v.push_back(T(0));
    v.push_back(T(1));
    v.push_back(T(2));
    return (v);
  }

#ifdef DEBUG_PARAMS
  static void print_params(const std::vector<double> &p, const CalibDataVec &cams) {
    using utils::rotation_matrix;
    using utils::vec_to_str;
    int off(0);
    std::cout << "------------------- parameters: -------------------" << std::endl;
    unsigned int intrinsics_offset(0);
    unsigned int extrinsics_base = get_num_intrinsics(cams);
    for(const auto cam_idx : irange<size_t>(0ul, cams.size())) {
      const CalibrationData &cam = cams[cam_idx];
      unsigned int off = intrinsics_offset;
      std::cout << "--------- camera # " << cam_idx << std::endl;
      std::cout << "fx: " << p[off]   << " fy: " << p[off+1] << std::endl;
      std::cout << "cx: " << p[off+2] << " cy: " << p[off+3] << std::endl;
      std::cout << "distortion coeff:";
      for(const auto k : irange(0u, (unsigned int)cam.intrinsics.distortion_coeffs.size())) {
        std::cout << " " << p[off + cam.intrinsics.intrinsics.size() + k];
      }
      std::cout << std::endl;
      if (cam_idx > 0) {
        off = extrinsics_base + 6 * (cam_idx - 1);
        const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
        Mat<double, 3, 3> R = rotation_matrix(rvec);
        const Vec<double, 3> t = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);
        //std::cout << "T_cam_cam0:  rot: " << "[" << vec_to_str(&R(0,0), 9) << "] trans: ["
        //<< vec_to_str(&t(0), 3) << "]" << std::endl;
      }
      intrinsics_offset += cam.intrinsics.intrinsics.size() +
        cam.intrinsics.distortion_coeffs.size();

    }
    off = extrinsics_base + 6 * (cams.size() - 1);
    std::cout << "--------- camera poses ----------------" << std::endl;
    for (; off < (int)p.size(); off += 6) {
#if 0
      const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
      Mat<double, 3, 3> R = rotation_matrix(rvec);
      const Vec<double, 3> t  = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);
      std::cout << "rot: " << "[" << vec_to_str(&R(0,0), 9) << "] trans: ["
                << vec_to_str(&t(0), 3) << "]" << std::endl;
#endif
    }
  }
#endif

  void Calibrator::addCamera(const std::string &name,
                             const CalibrationData &calibData) {
    calibrationData_.push_back(calibData);
    worldPoints_.resize(calibrationData_.size());
    imagePoints_.resize(calibrationData_.size());
  }

  void Calibrator::addPoints(int frameNum, const CamWorldPoints &wp, const CamImagePoints &ip,
                             const CameraExtrinsics &cam0PoseGuess) {
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
    cam0PoseGuess_.push_back(cam0PoseGuess);
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
    CameraExtrinsics T_cnm1_c0 = CameraExtrinsics::Identity();
    for(const auto cam_idx : irange(1u, num_cameras)) {
      // ------------ relative to camera 0 -----------------
      CameraExtrinsics T_cn_c0 = calibrationData_[cam_idx].T_cn_cnm1 * T_cnm1_c0;
      std::vector<double> rvec_tvec = transform_to_rvec_tvec(T_cn_c0);
      params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
      T_cnm1_c0 = T_cn_c0;
    }
    // the number of frames are the same across all cameras,
    // although not all frames must contain detected points
    const unsigned int num_frames = worldPoints_[0].size();
    // initialize pose guess for camera 0
    for(const auto i : irange(0u, num_frames))  {
      std::vector<double> rvec_tvec = transform_to_rvec_tvec(cam0PoseGuess_[i]);
      params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
    };
#ifdef DEBUG_PARAMS
    print_params(params, calibrationData_);
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
    print_params(params_, calibrationData_);
#endif
    std::cout << summary.FullReport() << std::endl;
  }

  void Calibrator::testCalibration(Residuals *resptr) {
    auto &res = *resptr; // residuals
    const unsigned int num_frames = worldPoints_[0].size();
    const unsigned int num_cameras = calibrationData_.size();

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
                                    calibrationData_);
      const unsigned numResiduals = 2 * frame_num_points;
      std::vector<double> residuals(numResiduals);
      // point offset to cam0 pose for this particular frame
      std::vector<double *> params = params_to_blocks(calibrationData_, params_, fnum);
      fr(&params[0], residuals.data());
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
    unsigned int numIntrinsics = get_num_intrinsics(calibrationData_);
    unsigned int intrinsics_offset = 0;

    CameraExtrinsics T_cnm1_0   = CameraExtrinsics::Identity();
    for (unsigned int cam_idx = 0; cam_idx < calibrationData_.size(); cam_idx++) {
      CalibrationData cd(calibrationData_[cam_idx]);
      CameraExtrinsics T_cn_0   = CameraExtrinsics::Identity();
      if (cam_idx > 0) {
        unsigned int off = numIntrinsics + 6 * (cam_idx - 1);
        const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
        Mat<double, 3, 3>       R = rotation_matrix(rvec);
        const Vec<double, 3>    t = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);

        T_cn_0.block<3,3>(0, 0) = R;
        T_cn_0.block<3,1>(0, 3) = t;
        T_cn_0(3,3) = 1.0;

        cd.T_cn_cnm1 = T_cn_0 * T_cnm1_0.inverse();

        T_cn_cam0 = cd.T_cn_cnm1 * T_cn_cam0;
        if (isNonZero(T_cam0_imu)) {
          cd.T_cam_imu = T_cn_0 * T_cam0_imu;
        }
      }

      T_cnm1_0 = T_cn_0;

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

    // number of intrinsic parameters for all cameras
    unsigned int numIntrinsics = get_num_intrinsics(calibrationData_);

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
        new FrameResidual(frame_world_points, frame_image_points, calibrationData_));
      std::vector<double *> v = params_to_blocks(calibrationData_, params, i);
      // Per camera intrinsics
      cost_function->AddParameterBlock(numIntrinsics);
      // (ncam-1) extrinsics
      for(unsigned int iext = 1; iext < calibrationData_.size(); iext++)  {
        cost_function->AddParameterBlock(6);
      }
      // Pose of cam0 wrt calib board
      cost_function->AddParameterBlock(6);

      // Reprojection error
      cost_function->SetNumResiduals(2 * frame_num_points);
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), v);

      // fix parameter blocks as requested
      if (fixIntrinsics_) {
        problem.SetParameterBlockConstant(v[0]);
      }
      for(const auto iext : irange(1ul, calibrationData_.size()))  {
        if (calibrationData_[iext].fixExtrinsics) {
          problem.SetParameterBlockConstant(v[iext]);
        }
      }
    }
  }
}
