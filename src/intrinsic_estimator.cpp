/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 *      Jason Owens jlowens+upenn@gmail.com
 */
#define DEBUG_PARAMS

#include "multicam_calibration/multicam_apriltag_detector.h"
#include <multicam_calibration/calibration_data.h>
#include <multicam_calibration/get_init_pose.h>
#include <multicam_calibration/utils.h>
#include <multicam_calibration/frame_residual.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <boost/range/irange.hpp>

#define CERES_PUBLIC_INTERNAL_CONFIG_H_
#include <ceres/ceres.h>


namespace multicam_calibration {
  using boost::irange;

  bool guessCameraPose(const FrameWorldPoints &wp,
                       const FrameImagePoints &ip,
                       CalibrationData& cd, 
                       CameraExtrinsics *T_0_w) {
    bool poseFound(false);
    if (!wp.empty()) {
      const CameraIntrinsics &ci = cd.intrinsics;
      CameraExtrinsics T_n_w = get_init_pose::get_init_pose(wp, ip,
                                                            ci.intrinsics,
                                                            ci.distortion_model,
                                                            ci.distortion_coeffs);
      ROS_INFO_STREAM("init pose: " << T_n_w);
      if (!poseFound) {
        // first camera with valid pose found
        *T_0_w = T_n_w;
        poseFound = true;
      }
    }
    return (poseFound);
  }

  std::ostream& operator<<(std::ostream& out, std::vector<double>& v)
  {
    out << "[";
    for (auto d : v) {
      out << d << ",";      
    }
    out << "]";
    return out;
  }
  
  void initializeVariables(std::vector<double>& params,
                           const CalibrationData& calibData,
                           std::vector<CameraExtrinsics>& exts) {
    // initialize intrinsics
    const CameraIntrinsics &ci = calibData.intrinsics;
    params.insert(params.end(), std::begin(ci.intrinsics),
                  std::end(ci.intrinsics));
    params.insert(params.end(), std::begin(ci.distortion_coeffs),
                  std::end(ci.distortion_coeffs));

    // the number of frames are the same across all cameras,
    // although not all frames must contain detected points
    const unsigned int num_frames = exts.size();
    // initialize pose guess for camera 0
    for(const auto i : irange(0u, num_frames))  {
      ROS_INFO_STREAM("ext:       " << exts[i]);
      std::vector<double> rvec_tvec = utils::transform_to_rvec_tvec(exts[i]);
      ROS_INFO_STREAM("rvec/tvec: " << rvec_tvec);
      params.insert(params.end(), rvec_tvec.begin(), rvec_tvec.end());
    }
  }

  void setupOptimizationProblem(const CamWorldPoints& cwp,
                                const CamImagePoints& cip,
                                CalibrationData& cd,
                                ceres::Problem *prob,
                                std::vector<double> *vars)
  {
    using CostFunction = ceres::DynamicAutoDiffCostFunction<FrameResidual>;
    std::vector<double> &params = *vars;
    ceres::Problem     &problem = *prob;
    
    const unsigned int num_frames = cwp.size();
    const unsigned int num_cameras = 1;

    unsigned int extrinsicsBase = 8; // 9 (fx,fy,cx,cy,d0...d3)
    // add the extrinsics between cam0 and the rest
    unsigned int totNumCameraParams = extrinsicsBase + 6 * (num_cameras - 1);

    //
    // Create one FrameResidual for each frame
    //    
    for(const auto i : irange(0u, num_frames))  {
      std::vector<FrameWorldPoints> frame_world_points;
      std::vector<FrameImagePoints> frame_image_points;
      unsigned int frame_num_points = 0;
      frame_world_points.push_back(cwp[i]);
      frame_image_points.push_back(cip[i]);
      frame_num_points = frame_world_points[0].size();
      CalibDataVec cds({cd});
      auto cost_function =
        new CostFunction(new FrameResidual(frame_world_points,
                                           frame_image_points,
                                           cds,
                                           extrinsicsBase));
      // Per camera intrinsics + (ncam-1) extrinsics
      cost_function->AddParameterBlock(totNumCameraParams);
      // Pose of cam0 wrt calib board
      cost_function->AddParameterBlock(6);
      // Reprojection error
      cost_function->SetNumResiduals(2 * frame_num_points);
      // point offset to cam0 pose for this particular frame
      const auto R_vec_offset = totNumCameraParams + 6 * i;
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                               &params[0], &params[R_vec_offset]);
    }
  }
  
  void runCalibration(std::vector<double>& params_,
                      const CamWorldPoints& cwp,
                      const CamImagePoints& cip,
                      CalibrationData& cd) {
    ceres::Problem problem;
    setupOptimizationProblem(cwp, cip, cd, &problem, &params_);
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

    std::cout << summary.FullReport() << std::endl;
#ifdef DEBUG_PARAMS
    CalibDataVec cdv({cd});
    utils::print_params(params_, cdv);
#endif
  }

  bool estimate_intrinsics(ros::NodeHandle& nh, size_t num_images, CalibrationData& data)
  {
    std::unique_ptr<MultiCamApriltagDetector>
      detector_(new MultiCamApriltagDetector(nh));
    // detect world/image points for num_images
    CamWorldPoints wp;
    CamImagePoints ip;
    //std::vector<sensor_msgs::Image::ConstPtr> msg_vec(imgs);
    // get images from the topic
    std::vector<sensor_msgs::Image::ConstPtr> imgs, single;
    ROS_INFO_STREAM("Listening for message on: " << data.rostopic);
    while (imgs.size() < num_images) {
      sensor_msgs::Image::ConstPtr img =
        ros::topic::waitForMessage<sensor_msgs::Image>(data.rostopic);
      if (img) {
        imgs.push_back(img);
        CamWorldPoints cwp;
        CamImagePoints cip;
        single.push_back(img);
        std::vector<apriltag_ros::ApriltagVec> detected_tags =
          detector_->process(single, &cwp, &cip);
        single.clear();
        if (detected_tags[0].size() == 0) {
          imgs.erase(imgs.end()-1);
        } else {
          ROS_INFO("Got %lu detections...", detected_tags[0].size());
          wp.push_back(cwp[0]);
          ip.push_back(cip[0]);
        }
      }
    }
    if (imgs.size() > 0) {

      // extract some initial values from the images
      int width = imgs[0]->width;
      int height  = imgs[0]->height;
      double cx = width/2.0;
      double cy = height/2.0;
      double fx = cx;
      double fy = cx;
      data.intrinsics.resolution = {width, height};
      data.intrinsics.intrinsics = {fx, fy, cx, cy};
      data.intrinsics.distortion_coeffs = {0.0,0.0,0.0,0.0}; // 4 params????


      std::vector<CameraExtrinsics> exts;
      for (size_t i = 0; i < wp.size(); ++i) {
        // assumes separate cameras
        // merge to single camera
        CameraExtrinsics cext;
        guessCameraPose(wp[i], ip[i], data, &cext);
        exts.push_back(cext);
      }

      std::vector<double> params;
      initializeVariables(params, data, exts);
#ifdef DEBUG_PARAMS
      CalibDataVec cds({data});
      utils::print_params(params, cds);
#endif
      runCalibration(params, wp, ip, data);

      // parse the params - fix the HACK
      data.intrinsics.intrinsics.assign(params.begin(),params.begin()+4);
      data.intrinsics.distortion_coeffs.assign(params.begin()+4, params.begin()+8);

#ifdef DEBUG_PARAMS
      utils::print_params(params, cds);
#endif
      return true;
    } else {
      ROS_WARN_STREAM("Received no images for topic " << data.rostopic);
      return false;
    }
  }  
}
