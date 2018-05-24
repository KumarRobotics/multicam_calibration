/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef MULTICAM_CALIBRATION_GET_INIT_POSE_H
#define MULTICAM_CALIBRATION_GET_INIT_POSE_H

#include "multicam_calibration/types.h"
#include <Eigen/Core>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace multicam_calibration {
  namespace get_init_pose {
    cv::Mat make_intrinsic_matrix(const std::vector<double> &intr);
    void    tf_to_rvec_tvec(const Eigen::Matrix<double, 4,4> &te,
                            cv::Affine3f::Vec3 *rvec, cv::Affine3f::Vec3 *tvec);
    Eigen::Matrix<double, 4, 4> get_init_pose(const std::vector<Point3<double>> &world_points,
                                              const std::vector<Point2<double>> &image_points,
                                              const std::vector<double> &intrinsics,
                                              const std::string &dist_model,
                                              const std::vector<double> &distcoeff = std::vector<double>());
    void project_points(const std::vector<Point3<double>> &wpe,
                        const Eigen::Matrix<double, 4,4> &Te,
                        const std::vector<double> &intrinsics,
                        const std::string &distModel,
                        const std::vector<double> &distcoeff,
                        std::vector<Point2<double>> *imagePoints);
  }
}

#endif
