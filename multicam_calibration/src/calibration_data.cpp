/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibration_data.h"

#include <yaml-cpp/yaml.h>
#include <boost/range/irange.hpp>
#include <fstream>

namespace multicam_calibration {
template <typename T>
static std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (const auto &i : v) {
    os << " " << i;
  }
  os << "]";
  return (os);
}

template <typename T>
static T get_param(const YAML::Node &node, const std::string &param,
                   const std::string camName) {
  if (!node[param]) {
    throw(std::runtime_error("cannot find " + param + " for cam " + camName));
  }
  T value;
  try {
    value = node[param].as<T>();
    return (value);
  } catch (const std::runtime_error &e) {
    std::cerr << "conversion failed for parameter " << param
              << " camera: " << camName << std::endl;
    throw(std::runtime_error("failed conversion param " + param + " for cam " +
                             camName));
  }
  return (value);  // should never be reached...
}

template <typename T>
static T get_param(const YAML::Node &node, const std::string &param,
                   const T &def, const std::string camName) {
  if (!node[param]) {
    return (def);
  }
  return (get_param<T>(node, param, camName));
}

template <>
CameraExtrinsics get_param(const YAML::Node &node, const std::string &param,
                           const CameraExtrinsics &def,
                           const std::string camName) {
  if (!node[param]) {
    return (def);
  }
  const auto &lines = node[param];
  if (lines.size() != 4) {
    std::cerr << "transform must have 4 lines for parameter " << param
              << " camera: " << camName << std::endl;
    throw(std::runtime_error("wrong num of transform lines"));
  }
  std::vector<double> vv;
  for (const auto l : boost::irange(0, 4)) {
    const auto &line = lines[l];
    auto v = line.as<std::vector<double>>();
    if (v.size() != 4) {
      std::cerr << "wrong num elements in line for tf " << param
                << " on cam: " << camName << std::endl;
      throw std::runtime_error("wrong num elems for cam extrinsics");
    }
    vv.insert(vv.end(), v.begin(), v.end());
  }
  CameraExtrinsics ce = Eigen::Map<Eigen::Matrix<double, 4, 4>>(vv.data());
  return (ce);
}

CalibDataVec CalibrationData::parse_cameras(const std::string &fname,
                                            CameraExtrinsics *T_imu_body) {
  std::cout << "parsing initial camera calibration file: " << fname
            << std::endl;
  std::ifstream fstream(fname);
  if (!fstream.is_open()) {
    std::cerr << "cannot find initial file: " << fname << std::endl;
    throw std::runtime_error("initial file not found!");
  }
  YAML::Node node = YAML::LoadFile(fname);
  CalibDataVec cdv;
  *T_imu_body = zeros();

  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const auto &node = it->second;
    CalibrationData calibData;
    calibData.name = name;
    CameraIntrinsics &ci = calibData.intrinsics;
    ci.camera_model = get_param<std::string>(node, "camera_model", name);
    ci.distortion_model =
        get_param<std::string>(node, "distortion_model", name);
    ci.distortion_coeffs =
        get_param<std::vector<double>>(node, "distortion_coeffs", name);
    ci.intrinsics = get_param<std::vector<double>>(node, "intrinsics", name);
    ci.resolution = get_param<std::vector<int>>(node, "resolution", name);
    calibData.rostopic = get_param<std::string>(node, "rostopic", name);
    calibData.fixExtrinsics =
        get_param<bool>(node, "fix_extrinsics", false, name);
    calibData.fixIntrinsics =
        get_param<bool>(node, "fix_intrinsics", false, name);
    calibData.active = get_param<bool>(node, "active", true, name);
    calibData.flipCode = get_param<int>(node, "flip_code", 2, name);
    calibData.rotateCode = get_param<int>(node, "rotate_code", -1, name);
    calibData.T_cam_imu =
        get_param<CameraExtrinsics>(node, "T_cam_imu", zeros(), name);
    calibData.T_cn_cnm1 =
        get_param<CameraExtrinsics>(node, "T_cn_cnm1", identity(), name);
    cdv.push_back(calibData);
  }
  return (cdv);
}

static std::ostream &print_intrinsics(std::ostream &os,
                                      const CameraIntrinsics &ci) {
  os << "camera mod: " << ci.camera_model << std::endl;
  os << "intrinsics: " << ci.intrinsics << std::endl;
  os << "dist model: " << ci.distortion_model << std::endl;
  os << "distortion: " << ci.distortion_coeffs << std::endl;
  os << "resolution: " << ci.resolution << std::endl;
  return (os);
}
std::ostream &operator<<(std::ostream &os, const CalibrationData &cd) {
  print_intrinsics(os, cd.intrinsics);
  return (os);
}
}  // namespace multicam_calibration
