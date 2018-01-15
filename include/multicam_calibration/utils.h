/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#ifndef MULTICAM_CALIBRATION_UTILS_H
#define MULTICAM_CALIBRATION_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/range/irange.hpp>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "types.h"

namespace multicam_calibration {
  namespace utils {
    template <typename T, int N>
      using Vec = Eigen::Matrix<T, N, 1>;

    template <typename T>
      using DynVec = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    
    template <typename T, int M, int N>
      using Mat = Eigen::Matrix<T, M, N>;
    
    template <typename T>
      Mat<T, 3, 3> rotation_matrix(const Vec<T, 3> &vec) {
      T angle = T(0);
      Vec<T, 3> axis{T(0), T(0), T(1)};
      const T vec_norm = vec.norm();
      if(vec_norm > 1e-8) {
        axis = vec / vec_norm;
        angle = vec_norm;
      }
      return Mat<T, 3, 3>(Eigen::AngleAxis<T>(angle, axis));
    }

    inline std::string vec_to_str(const double *p, int len) {
      std::stringstream ss;
      if(len > 0)  {
        ss << std::fixed << std::setw(9) << std::setprecision(5) << p[0];
        for(int i = 1; i < len; i++) {
          ss << ", " << std::fixed << std::setw(9) << std::setprecision(5) << p[i];
        }
      }
      return (ss.str());
    }

    inline std::string pose_to_str(const Pose &pose) {
      Vec<double, 3> rvec = {pose.first.x, pose.first.y, pose.first.z};
      Mat<double, 3, 3> rmat = rotation_matrix(rvec);
      double p[12] = {rmat(0,0), rmat(0,1), rmat(0,2),
                      rmat(1,0), rmat(1,1), rmat(1,2),
                      rmat(2,0), rmat(2,1), rmat(2,2),
                      pose.second.x, pose.second.y, pose.second.z};
      return (vec_to_str(p, 12));
    }
    
    template <typename T>
    Point2<T> project_point_equidistant(const Vec<T, 3> &X_world,
                                        const Mat<T, 3, 3> &R, const Vec<T, 3> &t,
                                        const Mat<T, 3, 3> &K,
                                        const DynVec<T> &D) {
      const Vec<T, 3> X_cam = R * X_world + t;
      const auto xy_squared_norm = X_cam(0) * X_cam(0) + X_cam(1) * X_cam(1);
      const auto hypot = xy_squared_norm != T{0} ? sqrt(xy_squared_norm) : T{0};
      const auto theta = atan2(hypot, X_cam(2));
      const auto psi =
        (X_cam(0) == T{0} && X_cam(1) == T{0}) ? T{0} : atan2(X_cam(1), X_cam(0));
      auto distortion_factor = T{1.0};
      for(auto i : boost::irange(0, (int)D.rows()))
        {
          distortion_factor += D[i] * pow(theta, 2 * (i + 1));
        }
      const auto r = theta * distortion_factor;
      const Vec<T, 3> x_dn{r * cos(psi), r * sin(psi), T{1.0}};
      const Vec<T, 3> x_dp = K * x_dn;
      return Point2<T>{x_dp(0), x_dp(1)};
    }

    template <typename T>
    std::vector<Point2<T>> project_frame_equidistant(
      const std::vector<Point3<double>> &world_points, const Mat<T, 3, 3> &R,
      const Vec<T, 3> &t, const Mat<T, 3, 3> &K, const DynVec<T> &D) {
      std::vector<Point2<T>> projected_points;
      projected_points.reserve(world_points.size());
      for(const auto point : world_points)
        {
          const auto pt = project_point_equidistant(
            Vec<T, 3>(T{point.x}, T{point.y}, T{point.z}), R, t, K, D);
          projected_points.push_back(pt);
        }
      return projected_points;
    }

    template <typename T>
    Point2<T> project_point_radtan(const Vec<T, 3> &X_world,
                                   const Mat<T, 3, 3> &R, const Vec<T, 3> &t,
                                   const Mat<T, 3, 3> &K,
                                   const DynVec<T> &D) {
      const Vec<T, 3> X = R * X_world + t;
      T x(X(0)/X(2));
      T y(X(1)/X(2));
      T x2  =  x * x;
      T y2  =  y * y;
      T xy  =  x * y;
      T rsq = x2 + y2;
      T r4  = rsq * rsq;
            
      T numerator = T{1.0} + D[0] * rsq + D[1] * r4;
      T denom     = T{1.0};

      if (D.size() > 4) {
        T r6  = rsq * r4;
        numerator += D[4] * r6;
        if (D.size() > 5) {
          denom += D[5] * rsq;
          if (D.size() > 6) {
            denom += D[6] * r4;
          }
          if (D.size() > 7) {
            denom += D[7] * r6;
          }
        }
      }
      T ratio = numerator / denom;
      T xpp = x * ratio + T{2.0} * D[2] * xy                  + D[3] * (rsq + T{2.0} * x2);
      T ypp = y * ratio + T{2.0} * D[2] * (rsq + T{2.0} * y2) + D[3] * xy;
                                   
      const Vec<T, 3> x_dn{xpp, ypp, T{1.0}};
      const Vec<T, 3> x_dp = K * x_dn;
      return Point2<T>{x_dp(0), x_dp(1)};
    }
 
    template <typename T>
    std::vector<Point2<T>> project_frame_radtan(
      const std::vector<Point3<double>> &world_points, const Mat<T, 3, 3> &R,
      const Vec<T, 3> &t, const Mat<T, 3, 3> &K, const DynVec<T> &D) {
      std::vector<Point2<T>> projected_points;
      projected_points.reserve(world_points.size());
      for(const auto point : world_points)
        {
          const auto pt = project_point_radtan(
            Vec<T, 3>(T{point.x}, T{point.y}, T{point.z}), R, t, K, D);
          projected_points.push_back(pt);
        }
      return projected_points;
    }

    inline std::vector<double>
    transform_to_rvec_tvec(const CameraExtrinsics &tf) {
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

    inline void
    rvec_tvec_to_transform(const std::vector<double>& rvec_tvec, CameraExtrinsics& ext)
    {
      const Vec<double,3> rvec = Eigen::Map<const Vec<double,3>>(&rvec_tvec[0]);
      const Vec<double,3> tvec = Eigen::Map<const Vec<double,3>>(&rvec_tvec[3]);
      Mat<double,3,3> R = rotation_matrix(rvec);
      ext = zeros();
      ext.block<3,3>(0,0) = R;
      ext.block<3,1>(0,3) = tvec;
      ext(3,3) = 1.0;      
    }

    inline std::vector<double>
    operator*(const std::vector<double>& v, double s)
    {
      std::vector<double> rv;
      for (double d : v) {
        rv.push_back(d * s);
      }
      return rv;
    }
    
    inline unsigned int
    get_extrinsics_base(const CalibDataVec &cdv) {
      unsigned int extrinsicsBase(0);
      for (const auto &cd : cdv) {
        // for each camera we have fx, fy, cx, cy + distortion coefficients;
        extrinsicsBase += cd.intrinsics.intrinsics.size() + cd.intrinsics.distortion_coeffs.size();
      }
      return (extrinsicsBase);
    }

    
#ifdef DEBUG_PARAMS  
    inline void print_params(const std::vector<double> &p, const CalibDataVec &cams) {
      
      using utils::rotation_matrix;
      using utils::vec_to_str;
      int off(0);
      std::cout << "------------------- parameters: -------------------" << std::endl;
      unsigned int intrinsics_offset(0);
      unsigned int extrinsics_base = get_extrinsics_base(cams);
      for(const auto cam_idx : boost::irange(0u, (unsigned int)cams.size())) {
        const CalibrationData &cam = cams[cam_idx];
        unsigned int off = intrinsics_offset;
        std::cout << "--------- camera # " << cam_idx << std::endl;
        std::cout << "fx: " << p[off]   << " fy: " << p[off+1] << std::endl;
        std::cout << "cx: " << p[off+2] << " cy: " << p[off+3] << std::endl;
        std::cout << "distortion coeff:";
        for(const auto k : boost::irange(0u, (unsigned int)cam.intrinsics.distortion_coeffs.size())) {
          std::cout << " " << p[off + cam.intrinsics.intrinsics.size() + k];
        }
        std::cout << std::endl;
        if (cam_idx > 0) {
          off = extrinsics_base + 6 * (cam_idx - 1);
          const Vec<double, 3> rvec = Eigen::Map<const Vec<double, 3>>(&p[off]);
          Mat<double, 3, 3> R = rotation_matrix(rvec);
          const Vec<double, 3> t = Eigen::Map<const Vec<double, 3>>(&p[off + 3]);
          std::cout << "T_cam_cam0:  rot: " << "[" << vec_to_str(&R(0,0), 9) << "] trans: ["
                    << vec_to_str(&t(0), 3) << "]" << std::endl;
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

    
  }
}

#endif
