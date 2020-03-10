/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibration_data.h"

namespace multicam_calibration {
  template<typename T>
  static std::ostream &operator<<(std::ostream &os, const std::vector<T>&v) {
    os << "[";
    for (const auto &i : v) {
      os << " " << i;
    }
    os << "]";
    return (os);
  }

  static void bombout(const std::string &param, const std::string &cam) {
    throw (std::runtime_error("cannot find " + param + " for cam " + cam));
  }

  static CameraExtrinsics get_kalibr_style_transform(const ros::NodeHandle &nh,
                                                     const std::string &field) {
    CameraExtrinsics T;
    XmlRpc::XmlRpcValue lines;
    if (!nh.getParam(field, lines)) {
      throw (std::runtime_error("cannot find transform " + field));
    }
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw (std::runtime_error("invalid transform " + field));
    }
    for (int i = 0; i < lines.size(); i++) {
      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw (std::runtime_error("bad line for transform " + field));
      }
      for (int j = 0; j < lines[i].size(); j++) {
        if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
          throw (std::runtime_error("bad value for transform " + field));
        } else {
          T(i, j) = static_cast<double>(lines[i][j]);
        }
      }
    }
    return (T);
  }

  static CameraExtrinsics get_transform(const ros::NodeHandle &nh, const std::string &field,
                                        const CameraExtrinsics &def) {
    CameraExtrinsics T(def);
    try {
      T = get_kalibr_style_transform(nh, field);
    } catch (std::runtime_error &e) {
    }
    return (T);
  }
  
  CalibDataVec CalibrationData::parse_cameras(const ros::NodeHandle &nh) {
    CalibDataVec cdv;
    std::vector<std::string> camNames = {"cam0", "cam1", "cam2"};
    for (const auto &cam : camNames) {
      XmlRpc::XmlRpcValue lines;
      if (!nh.getParam(cam, lines)) {
        continue;
      }
      CalibrationData calibData;
      calibData.name = cam;
      CameraIntrinsics &ci = calibData.intrinsics;
      if (!nh.getParam(cam + "/camera_model",
                       ci.camera_model)) { bombout("camera_model", cam); }
      if (!nh.getParam(cam + "/distortion_model",
                       ci.distortion_model)) { bombout("distortion_model", cam); }
      if (!nh.getParam(cam + "/distortion_coeffs",
                       ci.distortion_coeffs)) { bombout("distortion_coeffs", cam); }
      if (!nh.getParam(cam + "/intrinsics",  ci.intrinsics)) { bombout("intrinsics", cam); }
      if (!nh.getParam(cam + "/resolution",  ci.resolution)) { bombout("resolution", cam); }
      if (!nh.getParam(cam + "/rostopic",  calibData.rostopic)) { bombout("rostopic", cam); }
      calibData.T_cam_imu = get_transform(nh, cam + "/T_cam_imu", zeros());
      calibData.T_cn_cnm1 = get_transform(nh, cam + "/T_cn_cnm1", identity());
      nh.param<bool>(cam + "/fix_intrinsics",  calibData.fixIntrinsics, false);
      nh.param<bool>(cam + "/fix_extrinsics",  calibData.fixExtrinsics, false);
      nh.param<bool>(cam + "/active",          calibData.active, true);
      nh.param<int>(cam + "/flip_code",        calibData.flipCode, 2);
      nh.param<int>(cam + "/rotate_code",      calibData.rotateCode, -1);
      cdv.push_back(calibData);
    }
    return (cdv);
  }

  static std::ostream &print_intrinsics(std::ostream &os, const CameraIntrinsics &ci) {
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
}
