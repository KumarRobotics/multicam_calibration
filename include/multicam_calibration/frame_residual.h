#pragma once

#include <multicam_calibration/calibration_data.h>
#include <multicam_calibration/utils.h>
#include <multicam_calibration/types.h>
#include <ceres/ceres.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/range/irange.hpp>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Core>

#define CERES_PUBLIC_INTERNAL_CONFIG_H_
#include <ceres/ceres.h>

namespace multicam_calibration {
  using utils::Mat;
  using utils::Vec;
  using utils::DynVec;
  using utils::rotation_matrix;
  using boost::irange;

  struct FrameResidual
  {
    // Input is the list of world points and image points for the current frame in
    // each of the cameras
    FrameResidual(const std::vector<FrameWorldPoints> &world_points,
                  const std::vector<FrameImagePoints> &image_points,
                  const CalibDataVec &cams, unsigned int ext_base_off) :
      world_points_(world_points), image_points_(image_points), cams_(cams),
      extrinsics_base_offset_(ext_base_off)
    {
    }

    template <typename T>
    bool operator()(T const *const *params, T *residual) const
    {
      const Vec<T, 3> R_vec_frame = Eigen::Map<const Vec<T, 3>>(&params[1][0]);
      const Mat<T, 3, 3>  R_frame = rotation_matrix(R_vec_frame);
      const Vec<T, 3>     t_frame = Eigen::Map<const Vec<T, 3>>(&params[1][3]);

      unsigned int residual_count = 0;
      unsigned int intrinsics_offset = 0;
      for(const auto cam_idx : irange(0u, (unsigned int)cams_.size())) {        
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
        if(cam_idx == 0) {
          R_cam = Mat<T, 3, 3>::Identity();
          t_cam = Vec<T, 3>::Zero();
        }
        else {
          const auto extrinsic_start_idx = extrinsics_base_offset_ + 6 * (cam_idx - 1);
          const Vec<T, 3> R_vec_cam =
            Eigen::Map<const Vec<T, 3>>(&params[0][extrinsic_start_idx]);
          R_cam = rotation_matrix(R_vec_cam); // rotation vector
          t_cam = 
            Eigen::Map<const Vec<T, 3>>(&params[0][extrinsic_start_idx + 3]);
        }
                  
        const Mat<T, 3, 3> R = R_cam * R_frame;
        const Vec<T, 3> t = R_cam * t_frame + t_cam;
        std::vector<Point2<T>> projected_points;
        if (cam.intrinsics.distortion_model == "equidistant") {
          projected_points = utils::project_frame_equidistant(world_points_[cam_idx], R, t, K, D);
        } else if (cam.intrinsics.distortion_model == "radtan") {
          projected_points = utils::project_frame_radtan(world_points_[cam_idx], R, t, K, D);
        } else {
          std::cout << "ERROR: unknown distortion model: " << cam.intrinsics.distortion_model << std::endl;
          return (false);
        }
        for(const auto i : irange<size_t>(0, projected_points.size())) {
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
    unsigned int extrinsics_base_offset_;
  };
}
