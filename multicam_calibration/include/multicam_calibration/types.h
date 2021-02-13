/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */

#ifndef MULTICAM_CALIBRATION_TYPES_H
#define MULTICAM_CALIBRATION_TYPES_H
#include <ostream>
#include <vector>
#include <Eigen/Core>

namespace multicam_calibration {

  template <typename T>
  using vector_aligned = std::vector<T, Eigen::aligned_allocator<T>>;

  template <typename T>
  struct Point2 {
    T x, y;
    Point2(const T _x, const T _y) : x(_x), y(_y) {}

    // Requires ths since it can be called with T = ceres::Jet which contains an
    // Eigen::Matrix
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend std::ostream &operator<<(std::ostream &stream,
                                    const Point2<T> &pt) {
      stream << "(" << pt.x << ", " << pt.y << ")";
      return stream;
    }
  };

  template <typename T>
  struct Point3 {
    T x, y, z;
    Point3(const T _x, const T _y, const T _z) : x(_x), y(_y), z(_z) {}

    // Requires ths since it can be called with T = ceres::Jet which contains an
    // Eigen::Matrix
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    friend std::ostream &operator<<(std::ostream &stream,
                                    const Point3<T> &pt) {
      stream << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")";
      return stream;
    }
  };

  using FrameWorldPoints = std::vector<Point3<double>>;
  using FrameImagePoints = std::vector<Point2<double>>;

  using CamWorldPoints = std::vector<FrameWorldPoints>;
  using CamImagePoints = std::vector<FrameImagePoints>;

  using Pose = std::pair<Point3<double>, Point3<double>>;
}
#endif
