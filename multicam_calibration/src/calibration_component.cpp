/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibration_component.h"
#include "multicam_calibration/camera_extrinsics.h"
#include "multicam_calibration/get_init_pose.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <boost/range/irange.hpp>
#include <cassert>
#include <chrono>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>

namespace multicam_calibration {
using boost::irange;

static std::string make_filename(const std::string &base,
                                 const std::string &ext = ".yaml") {
  using std::chrono::system_clock;
  std::time_t tt = system_clock::to_time_t(system_clock::now());
  struct std::tm *ptm = std::localtime(&tt);
  std::stringstream ss;
  ss << std::put_time(ptm, "%F-%H-%M-%S");
  return (base + "-" + ss.str() + ext);
}

CalibrationComponent::CalibrationComponent(const rclcpp::NodeOptions &options)
    : Node("calibration", options) {
  initialize();
}

void CalibrationComponent::initialize() {
  cameras_ready_ = false;
  calibrator_.reset(new Calibrator());
  deviceName_ = declare_parameter("device_name", "device_name");
  calibDir_ = declare_parameter("calib_dir", "calib");
  try {
    const std::string calibFile = declare_parameter(
        "calibration_file", calibDir_ + "/" + deviceName_ + "-initial.yaml");
    parseCameras(calibFile);
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR_STREAM(get_logger(), "error parsing cameras: " << e.what());
    rclcpp::shutdown();
  }
  use_approximate_sync_ = declare_parameter("use_approximate_sync", true);
  RCLCPP_INFO_STREAM(get_logger(), (use_approximate_sync_ ? "" : "not ")
                                       << "using approximate sync");
  const std::string cornersInFile = declare_parameter("corners_in_file", "");
  const std::string cornersOutFile =
      declare_parameter("corners_out_file", calibDir_ + "/corners.csv");
  const int detectorType = declare_parameter("detector", 0);
  const std::string family = declare_parameter("tag_family", "36h11");
  const int blackBorder = declare_parameter("black_border", 2);
  const int tagCols = declare_parameter("tag_cols", 7);
  const int tagRows = declare_parameter("tag_rows", 5);
  const float tagSize = declare_parameter("tag_size", 0.04);
  const float tagSpacing = declare_parameter("tag_spacing", 0.25);
  const std::string targetType = declare_parameter("target_type", "aprilgrid");

  if (targetType != "aprilgrid") {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid target_type: " << targetType);
    throw std::invalid_argument("target_type != aprilgrid");
  }

  detector_.reset(new MultiCamApriltagDetector(
      this, family, detectorType, blackBorder, tagRows, tagCols, tagSize,
      tagSpacing, cornersOutFile));
  skipFrames_ = declare_parameter("skip_num_frames", 1);
  const bool fixIntrinsics = declare_parameter("fix_intrinsics", false);
  calibrator_->setFixAllIntrinsics(fixIntrinsics);
  record_bag_ = declare_parameter("record_bag", false);
  bag_file_ = declare_parameter("bag_base", "~/.ros/multicam_calibration");
  if (record_bag_) {
    RCLCPP_ERROR_STREAM(get_logger(), "bag writing not supported under ROS2!");
    throw std::runtime_error("bag writing not supported!");
    auto bagDir = rcpputils::fs::path(bag_file_);
    rcpputils::fs::remove_all(bagDir);
    // bagWriter_.reset(new rosbag2_cpp::Writer());
    // bagWriter_->open(bagDir.string());
    // bagIsOpen_ = true;
  }

  for (const auto &camdata : cameras_) {
    const auto qos = rmw_qos_profile_default;
    if (cameras_.size() > 1) {
      sub_.push_back(std::make_shared<message_filters::Subscriber<ImageMsg>>(
          this, camdata.rostopic, qos));
    } else {
      singleCamSub_ = image_transport::create_subscription(
          this, camdata.rostopic,
          std::bind(&CalibrationComponent::callback1, this,
                    std::placeholders::_1),
          "raw", qos);
    }
    imagePub_.push_back(image_transport::create_publisher(
        this, camdata.name + "/debug_image", qos));
    tagCountPub_.push_back(create_publisher<std_msgs::msg::UInt32>(
        camdata.name + "/num_detected_tags", 10));
  }
  subscribe();
  //
  calibrationService_ = create_service<std_srvs::srv::Trigger>(
      "calibration", std::bind(&CalibrationComponent::calibrate, this,
                               std::placeholders::_1, std::placeholders::_2));
  parameterService_ =
      create_service<multicam_calibration_srvs::srv::ParameterCmd>(
          "set_parameter",
          std::bind(&CalibrationComponent::setParameter, this,
                    std::placeholders::_1, std::placeholders::_2));
  if (!cornersInFile.empty()) {
    if (readPointsFromFile(cornersInFile)) {
      if (!worldPoints_.empty()) {
        RCLCPP_INFO_STREAM(get_logger(), "read " << worldPoints_[0].size()
                                                 << " frames from "
                                                 << cornersInFile);
        const bool runCalibOnInit =
            declare_parameter("run_calib_on_init", false);
        if (runCalibOnInit) {
          std::shared_ptr<std_srvs::srv::Trigger::Request> rq(
              new std_srvs::srv::Trigger::Request());
          std::shared_ptr<std_srvs::srv::Trigger::Response> rsp(
              new std_srvs::srv::Trigger::Response());
          calibrate(rq, rsp);
          rclcpp::shutdown();
        }
      }
    } else {
      RCLCPP_ERROR_STREAM(get_logger(),
                          "file not found or bad: " << cornersInFile);
    }
  }
}

void CalibrationComponent::parseCameras(const std::string &fname) {
  cameras_ = CalibrationData::parse_cameras(fname, &T_imu_body_);
  worldPoints_.resize(cameras_.size());
  imagePoints_.resize(cameras_.size());
  calibrator_->setCameras(cameras_);
  RCLCPP_INFO_STREAM(get_logger(), "Found " << cameras_.size() << " cameras!");
  cameras_ready_ = true;
}

bool CalibrationComponent::calibrate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  calibrator_->setCameras(cameras_);
  calibrator_->showCameraStatus();
  calibrator_->runCalibration();
  CalibDataVec results = calibrator_->getCalibrationResults();
  if (results.empty()) {
    RCLCPP_ERROR(get_logger(), "empty calibration results, no file written!");
    res->message = "no data for calibration!";
    res->success = false;
    return (true);
  }

  const std::string linkName =
      declare_parameter("latest_link_name", "latest.yaml");
  const std::string resultsDir = declare_parameter("results_dir", ".");

  std::string fname = resultsDir + "/" + make_filename(deviceName_);
  std::string fullname = calibDir_ + "/" + fname;

  RCLCPP_INFO_STREAM(get_logger(), "writing calibration to " << fullname);
  std::ofstream of(fullname);
  updateCameras(results);
  calibrator_->setCameras(cameras_);
  writeCalibration(of, results);
  writeCalibration(std::cout, results);
  // test with poses from optimizer
  std::string calibErr = testCalibration(results);
  std::string cmd = "ln -sf " + fname + " " + calibDir_ + "/" + linkName;
  if (std::system(NULL) && std::system(cmd.c_str())) {
    RCLCPP_ERROR_STREAM(get_logger(), "link command failed: " << cmd);
  }

  // close the bag
  if (record_bag_ && bagIsOpen_) {
    bagIsOpen_ = false;
  }
  res->success = true;
  res->message = calibErr;

  return (true);
}

int CalibrationComponent::getCameraIndex(const std::string &cam) const {
  for (const auto cam_idx : irange(0ul, cameras_.size())) {
    if (cameras_[cam_idx].name == cam) return (cam_idx);
  }
  RCLCPP_ERROR_STREAM(get_logger(), "invalid camera specified: " << cam);
  throw(std::runtime_error("invalid camera specified: " + cam));
  return (-1);
}

bool CalibrationComponent::setParameter(
    const std::shared_ptr<multicam_calibration_srvs::srv::ParameterCmd::Request>
        req,
    std::shared_ptr<multicam_calibration_srvs::srv::ParameterCmd::Response>
        res) {
  int cam_idx = getCameraIndex(req->camera);
  res->success = true;
  switch (req->param) {
    case 0:  // fix intrinsics
      cameras_[cam_idx].fixIntrinsics = req->value;
      res->message = "fix intrinsics successful";
      break;
    case 1:  // fix extrinsics
      cameras_[cam_idx].fixExtrinsics = req->value;
      res->message = "fix extrinsics successful";
      break;
    case 2:  // set active/inactive
      cameras_[cam_idx].active = req->value;
      res->message = "set active successful";
      break;
    case 3:  // exit calibration
      res->message = "exiting";
      rclcpp::shutdown();
      break;
    default:
      RCLCPP_ERROR_STREAM(get_logger(),
                          "invalid parameter: " << (int)req->param);
      res->success = false;
      res->message = "invalid parameter";
      return (false);
  }
  return (true);
}

typedef std::pair<double, unsigned int> Stat;
static double avg(const Stat &s) {
  return (s.second > 0 ? s.first / s.second : 0.0);
}

std::string CalibrationComponent::testCalibration(const CalibDataVec &calib) {
  Calibrator::Residuals res;
  calibrator_->testCalibration(&res);
  if (res.size() == 0) {
    RCLCPP_ERROR(get_logger(), "no residuals found for test!");
    return (std::string("calibration failed: no data found!"));
  }
  Stat totErr;
  Stat maxErr;
  unsigned int maxErrCam(0);
  std::vector<Stat> cameraStats(calib.size(), Stat(0.0, 0));
  std::vector<std::ofstream> resFiles;
  for (const auto cam_idx : irange(0ul, calib.size())) {
    resFiles.push_back(std::ofstream(calibDir_ + "/residuals_cam" +
                                     std::to_string(cam_idx) + ".txt"));
  }
  for (const auto fnum : irange(0ul, res.size())) {
    for (const auto cam_idx : irange(0ul, res[fnum].size())) {
      for (const auto res_idx : irange(0ul, res[fnum][cam_idx].size())) {
        Calibrator::Vec2d v = res[fnum][cam_idx][res_idx];
        double err = v(0) * v(0) + v(1) * v(1);
        totErr.first += err;
        totErr.second++;
        cameraStats[cam_idx].first += err;
        cameraStats[cam_idx].second++;
        const auto &wp = worldPoints_[cam_idx][fnum][res_idx];
        const auto &ip = imagePoints_[cam_idx][fnum][res_idx];
        Calibrator::Vec2d dp(ip.x - calib[cam_idx].intrinsics.intrinsics[2],
                             ip.y - calib[cam_idx].intrinsics.intrinsics[3]);
        resFiles[cam_idx] << wp.x << " " << wp.y << " " << ip.x << " " << ip.y
                          << " " << dp(0) << " " << dp(1) << " "
                          << std::sqrt(dp(0) * dp(0) + dp(1) * dp(1)) << " "
                          << std::sqrt(err) << " " << v(0) << " " << v(1)
                          << std::endl;
        if (err > maxErr.first) {
          maxErr.first = err;
          maxErr.second = fnum;
          maxErrCam = cam_idx;
        }
      }
    }
  }
  RCLCPP_INFO(get_logger(),
              "----------------- reprojection errors: ---------------");
  RCLCPP_INFO_STREAM(get_logger(),
                     "total error:     " << std::sqrt(avg(totErr)) << " px");
  std::stringstream ss;
  ss << "calib proj err: ";
  for (const auto cam_idx : irange(0ul, cameraStats.size())) {
    const double pxErr = std::sqrt(avg(cameraStats[cam_idx]));
    RCLCPP_INFO_STREAM(get_logger(),
                       "avg error cam " << cam_idx << ": " << pxErr << " px");
    ss << "cam" << cam_idx << ": " << pxErr << "px ";
  }
  RCLCPP_INFO_STREAM(get_logger(),
                     "max error: " << std::sqrt(maxErr.first)
                                   << " px at frame: " << maxErr.second
                                   << " for cam: " << maxErrCam);
  return (ss.str());
}

static std::string vec2str(const std::vector<double> v) {
  std::stringstream ss;
  ss << "[";
  if (v.size() > 0) {
    ss << v[0];
  }
  for (unsigned int i = 1; i < v.size(); i++) {
    ss << ", " << v[i];
  }
  ss << "]";
  return (ss.str());
}

template <typename T>
static void print_tf(std::ostream &os, const std::string &p, const T &tf) {
  os << std::fixed << std::setprecision(11);
  const int w = 14;
  for (unsigned int i = 0; i < tf.rows(); i++) {
    os << p << "- [";
    for (unsigned int j = 0; j < tf.cols() - 1; j++) {
      os << std::setw(w) << tf(i, j) << ", ";
    }
    os << std::setw(w) << tf(i, tf.cols() - 1) << "] " << std::endl;
  }
}

static bool compute_RPQ(unsigned int cam1_idx, unsigned int cam2_idx,
                        const CalibDataVec &calib, cv::Mat *R1, cv::Mat *R2,
                        cv::Mat *P1, cv::Mat *P2, cv::Mat *Q) {
  // only compute stereo transformations for the first two cameras
  if (cam1_idx >= calib.size() || cam2_idx >= calib.size()) {
    return (false);
  }
  const auto &ci1 = calib[cam1_idx].intrinsics;
  const auto &ci2 = calib[cam2_idx].intrinsics;
  const auto &distm = ci1.distortion_model;
  // they must have same distortion model and resolution
  if (distm != ci2.distortion_model || ci1.resolution[0] != ci2.resolution[0] ||
      ci1.resolution[1] != ci2.resolution[1]) {
    return (false);
  }
  const cv::Mat K1 = get_init_pose::make_intrinsic_matrix(ci1.intrinsics);
  const cv::Mat K2 = get_init_pose::make_intrinsic_matrix(ci2.intrinsics);
  cv::Affine3f::Vec3 rvec, tvec;
  get_init_pose::tf_to_rvec_tvec(calib[cam2_idx].T_cn_cnm1, &rvec, &tvec);

  if (distm == "equidistant") {
    cv::fisheye::stereoRectify(
        K1, ci1.distortion_coeffs, K2, ci2.distortion_coeffs,
        cv::Size(ci1.resolution[0], ci2.resolution[1]), rvec, tvec, *R1, *R2,
        *P1, *P2, *Q, cv::CALIB_ZERO_DISPARITY);
  } else if (distm == "radtan" || distm == "plumb_bob") {
    // XXX never tested!
    cv::stereoRectify(K1, ci1.distortion_coeffs, K2, ci2.distortion_coeffs,
                      cv::Size(ci1.resolution[0], ci2.resolution[1]), rvec,
                      tvec, *R1, *R2, *P1, *P2, *Q);
  } else {
    throw(std::runtime_error("unknown distortion model: " + distm));
  }
  return (true);
}

void CalibrationComponent::writeCalibration(std::ostream &os,
                                            const CalibDataVec &results) {
  cv::Mat R[2], P[2], Q;
  const unsigned int STEREO_CAM_0 = 0;
  const unsigned int STEREO_CAM_1 = 1;
  bool hasRP = compute_RPQ(STEREO_CAM_0, STEREO_CAM_1, results, &R[0], &R[1],
                           &P[0], &P[1], &Q);
  for (unsigned int cam_idx = 0; cam_idx < results.size(); cam_idx++) {
    const CalibrationData &cd = results[cam_idx];
    os << cd.name << ":" << std::endl;
    if (isNonZero(cd.T_cam_imu)) {
      os << "  T_cam_imu:" << std::endl;
      print_tf<CameraExtrinsics>(os, "  ", cd.T_cam_imu);
    }
    if (cam_idx > 0) {
      os << "  T_cn_cnm1:" << std::endl;
      print_tf<CameraExtrinsics>(os, "  ", cd.T_cn_cnm1);
    }
    if (hasRP && (cam_idx == STEREO_CAM_0 || cam_idx == STEREO_CAM_1)) {
      unsigned int rp_idx = cam_idx == STEREO_CAM_0 ? 0 : 1;
      Eigen::Matrix<double, 3, 3> Re;
      Eigen::Matrix<double, 3, 4> Pe;
      cv::cv2eigen(R[rp_idx], Re);
      cv::cv2eigen(P[rp_idx], Pe);
      os << "  rectification_matrix:" << std::endl;
      print_tf<Eigen::Matrix<double, 3, 3>>(os, "  ", Re);
      os << "  projection_matrix:" << std::endl;
      print_tf<Eigen::Matrix<double, 3, 4>>(os, "  ", Pe);
    }
    const CameraIntrinsics &ci = cd.intrinsics;
    os << "  camera_model: " << ci.camera_model << std::endl;
    os << "  intrinsics: " << vec2str(ci.intrinsics) << std::endl;
    os << "  distortion_model: " << ci.distortion_model << std::endl;
    os << "  distortion_coeffs: " << vec2str(ci.distortion_coeffs) << std::endl;
    os << "  resolution: [" << ci.resolution[0] << ", " << ci.resolution[1]
       << "]" << std::endl;
    os << "  rostopic: " << cd.rostopic << std::endl;
  }
  if (isNonZero(T_imu_body_)) {
    os << "T_imu_body:" << std::endl;
    print_tf<CameraExtrinsics>(os, "  ", T_imu_body_);
  }
}

void CalibrationComponent::updateCameras(const CalibDataVec &results) {
  for (const auto &cam_idx : irange(0ul, results.size())) {
    const CalibrationData &cd = results[cam_idx];
    const CameraIntrinsics &ci = cd.intrinsics;
    CalibrationData &cam = cameras_[cam_idx];
    if (isNonZero(cd.T_cam_imu)) {
      cam.T_cam_imu = cd.T_cam_imu;
    }
    if (cam_idx > 0) {
      cam.T_cn_cnm1 = cd.T_cn_cnm1;
    }
    cam.intrinsics.intrinsics = ci.intrinsics;
    cam.intrinsics.distortion_coeffs = ci.distortion_coeffs;
  }
}

void CalibrationComponent::subscribe() {
  switch (sub_.size()) {
    case 0:
      break;
    case 2:
      if (use_approximate_sync_) {
        approx_sync2_.reset(new ApproxTimeSynchronizer2(
            ApproxSyncPolicy2(60 /*q size*/), *(sub_[0]), *(sub_[1])));
        approx_sync2_->registerCallback(&CalibrationComponent::callback2, this);
      } else {
        sync2_.reset(new ExactSynchronizer2(*(sub_[0]), *(sub_[1]), 2));
        sync2_->registerCallback(&CalibrationComponent::callback2, this);
      }
      break;
    case 3:
      if (use_approximate_sync_) {
        approx_sync3_.reset(
            new ApproxTimeSynchronizer3(ApproxSyncPolicy3(60 /*q size*/),
                                        *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approx_sync3_->registerCallback(&CalibrationComponent::callback3, this);
      } else {
        sync3_.reset(
            new ExactSynchronizer3(*(sub_[0]), *(sub_[1]), *(sub_[2]), 2));
        sync3_->registerCallback(&CalibrationComponent::callback3, this);
      }
      break;
    case 4:
      if (use_approximate_sync_) {
        approx_sync4_.reset(new ApproxTimeSynchronizer4(
            ApproxSyncPolicy4(60 /*q size*/), *(sub_[0]), *(sub_[1]),
            *(sub_[2]), *(sub_[3])));
        approx_sync4_->registerCallback(&CalibrationComponent::callback4, this);
      } else {
        RCLCPP_ERROR(get_logger(), "No exact sync beyond 3 cameras, right now");
      }
      break;
    default:
      RCLCPP_ERROR(get_logger(), "invalid number of subscribers!");
  }
}

bool CalibrationComponent::guessCameraPose(const CamWorldPoints &wp,
                                           const CamImagePoints &ip,
                                           CameraExtrinsics *T_0_w,
                                           int frameNum) const {
  CameraExtrinsics T_n_0 = identity();
  bool poseFound(false);
  for (unsigned int cam_idx = 0; cam_idx < cameras_.size(); cam_idx++) {
    const CalibrationData &cd = cameras_[cam_idx];
    T_n_0 = cd.T_cn_cnm1 * T_n_0;  // chain forward from cam0 to current camera
    if (!wp[cam_idx].empty()) {
      const CameraIntrinsics &ci = cd.intrinsics;
      CameraExtrinsics T_n_w = get_init_pose::get_init_pose(
          wp[cam_idx], ip[cam_idx], ci.intrinsics, ci.distortion_model,
          ci.distortion_coeffs);
      // std::cout << cam_idx << " init pose: " << std::endl << T_n_w <<
      // std::endl;
      if (!poseFound) {
        // first camera with valid pose found
        *T_0_w = T_n_0.inverse() * T_n_w;
        poseFound = true;
      } else {
        CameraExtrinsics T_0_w_test = T_n_0.inverse() * T_n_w;
        CameraExtrinsics T_err = T_0_w_test.inverse() * (*T_0_w);
        double rot_err = 1.5 - 0.5 * (T_err(0, 0) + T_err(1, 1) + T_err(2, 2));
        if (rot_err > 0.25) {
          RCLCPP_WARN_STREAM(get_logger(),
                             "init tf for camera "
                                 << cam_idx << " frame " << frameNum
                                 << " is off from your initial .yaml file!");
          RCLCPP_WARN_STREAM(get_logger(), "expected T_n_0 from "
                                               << cam_idx
                                               << " to cam 0 is roughly:");
          RCLCPP_WARN_STREAM(get_logger(), T_n_w * T_0_w->inverse());
        }
      }
    }
  }
  return (poseFound);
}

static bool get_intrinsics_csv_data(const std::string &filename,
                                    const unsigned int camera_id,
                                    std::vector<int> *frame_nums,
                                    CamWorldPoints &world_points,
                                    CamImagePoints &img_pts,
                                    int skip_factor = 0) {
  std::ifstream in(filename);
  if (!in.is_open()) return false;

  std::string line;

  int frame_num;
  int prev_frame_num = -1;
  unsigned int frame_count = 0;
  frame_nums->clear();
  while (getline(in, line)) {
    float world_x, world_y, cam_x, cam_y;
    unsigned int cam_idx;
    if (sscanf(line.c_str(), "%d, intrinsics, %u, %f, %f, %f, %f", &frame_num,
               &cam_idx, &world_x, &world_y, &cam_x, &cam_y) != 6)
      continue;
    if (cam_idx != camera_id) continue;
    if (frame_num % (skip_factor + 1) != 0) continue;

    bool new_frame = false;
    if (frame_num != prev_frame_num) {
      prev_frame_num = frame_num;
      new_frame = true;
      ++frame_count;
    }

    if (new_frame) {
      world_points.push_back(FrameWorldPoints());
      frame_nums->push_back(frame_num);
    }

    world_points[frame_count - 1].emplace_back(world_x, world_y, 0);

    if (new_frame) img_pts.push_back(FrameImagePoints());

    img_pts[frame_count - 1].emplace_back(cam_x, cam_y);
  }
  in.close();
  printf("read %u frames from corners file for cam %u\n", frame_count,
         camera_id);
  return true;
}

bool CalibrationComponent::readPointsFromFile(const std::string &fname) {
  std::set<int> fnum_set;
  for (unsigned int camid = 0; camid < cameras_.size(); camid++) {
    std::vector<int> fnums;
    CamWorldPoints wp;
    CamImagePoints ip;
    if (!get_intrinsics_csv_data(fname, camid, &fnums, wp, ip)) {
      return (false);
    }
    std::copy(fnums.begin(), fnums.end(),
              std::inserter(fnum_set, fnum_set.end()));
    RCLCPP_INFO_STREAM(get_logger(),
                       "camid " << camid << " read frames: " << fnums.size());
  }
  RCLCPP_INFO_STREAM(get_logger(),
                     "total number of frames: " << fnum_set.size());
  for (unsigned int camid = 0; camid < cameras_.size(); camid++) {
    std::vector<int> fnums;
    CamWorldPoints wp;
    CamImagePoints ip;
    if (!get_intrinsics_csv_data(fname, camid, &fnums, wp, ip)) {
      return (false);
    }
    worldPoints_[camid].resize(fnum_set.size());
    imagePoints_[camid].resize(fnum_set.size());
    for (unsigned int i = 0; i < fnums.size(); i++) {
      int fnum = fnums[i];
      int frank = std::distance(fnum_set.begin(), fnum_set.find(fnum));
      worldPoints_[camid][frank] = wp[i];
      imagePoints_[camid][frank] = ip[i];
    }
  }

  for (unsigned int fnum = 0; fnum < worldPoints_[0].size(); fnum++) {
    CamWorldPoints wp;
    CamImagePoints ip;
    for (unsigned int camid = 0; camid < cameras_.size(); camid++) {
      wp.push_back(worldPoints_[camid][fnum]);
      ip.push_back(imagePoints_[camid][fnum]);
    }
    CameraExtrinsics cam0Pose;
    if (!guessCameraPose(wp, ip, &cam0Pose, fnum)) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "no detections found, skipping frame " << fnum);
      continue;
    }
    calibrator_->addPoints(frameNum_, wp, ip, cam0Pose);
  }
  return (true);
}

void CalibrationComponent::process(const std::vector<ImageConstPtr> &msg_vec) {
  if ((skipCount_++ % skipFrames_) != 0) {
    return;
  }
  CamWorldPoints wp;
  CamImagePoints ip;

  std::vector<apriltag_ros::ApriltagVec> detected_tags =
      detector_->process(msg_vec, &wp, &ip);
  assert(msg_vec.size() == detected_tags.size());
  // Insert newly detected points into existing set.
  // At this point one could add the data points to an
  // incremental solver.
  CameraExtrinsics cam0Pose;
  if (!guessCameraPose(wp, ip, &cam0Pose, frameNum_)) {
    RCLCPP_WARN_STREAM(get_logger(), "no detections found, skipping frame ");
    publishDebugImages(msg_vec, detected_tags);
    return;
  }

  if (record_bag_ && bagIsOpen_) {
    for (size_t i = 0; i < cameras_.size(); ++i) {
      // auto &img = msg_vec[i];

      // bag_writer_->write(cameras_[i].rostopic, img->header.stamp, *img);
    }
  }

  calibrator_->addPoints(frameNum_, wp, ip, cam0Pose);

  // add new frames to each camera
  for (unsigned int cam_idx = 0; cam_idx < wp.size(); cam_idx++) {
    if (cam_idx < wp.size()) {
      worldPoints_[cam_idx].push_back(wp[cam_idx]);
    }
    if (cam_idx < ip.size()) {
      imagePoints_[cam_idx].push_back(ip[cam_idx]);
    }
  }
  CalibDataVec::iterator cam = cameras_.begin();
  for (const auto &tags : detected_tags) {
    cam->tagCount += tags.size();
    ++cam;
    if (cam == cameras_.end()) {
      break;
    }
  }
  publishDebugImages(msg_vec, detected_tags);
  publishTagCounts();
  frameNum_++;
}

cv::Mat CalibrationComponent::flipRotate(const cv::Mat &img, int camIdx) const {
  // flip or rotate image if requested
  const CalibrationData &cd = cameras_[camIdx];
  const int rotateCode = cd.rotateCode;
  cv::Mat rotFlipImg = img;
  if (rotateCode != -1 && rotateCode <= 2) {
    cv::rotate(img, rotFlipImg, rotateCode);
  }
  const int flipCode = cd.flipCode;
  if (abs(flipCode) <= 1) {
    cv::flip(rotFlipImg, rotFlipImg, flipCode);
  }
  return (rotFlipImg);
}

void CalibrationComponent::publishDebugImages(
    const std::vector<ImageMsg::ConstSharedPtr> &msg_vec,
    const std::vector<apriltag_ros::ApriltagVec> &detected_tags) {
  for (int i = 0; i < (int)detected_tags.size(); i++) {
    if (imagePub_[i].getNumSubscribers() > 0) {
      cv_bridge::CvImageConstPtr const cv_ptr =
          cv_bridge::toCvShare(msg_vec[i], sensor_msgs::image_encodings::MONO8);
      const cv::Mat gray = cv_ptr->image;
      if (gray.rows == 0) {
        RCLCPP_ERROR(get_logger(), "cannot decode image, not MONO8!");
        continue;
      }
      cv::Mat img;
      cv::cvtColor(gray, img, CV_GRAY2BGR);
      apriltag_ros::DrawApriltags(img, detected_tags[i]);
      cv::Mat rotImg = flipRotate(img, i);
      cv_bridge::CvImage cv_img(msg_vec[i]->header,
                                sensor_msgs::image_encodings::BGR8, rotImg);
      imagePub_[i].publish(cv_img.toImageMsg());
    }
  }
}

void CalibrationComponent::publishTagCounts() {
  for (unsigned int i = 0; i < cameras_.size(); i++) {
    const CalibrationData &cd = cameras_[i];
    if (tagCountPub_[i]->get_subscription_count() > 0) {
      std_msgs::msg::UInt32 msg;
      msg.data = cd.tagCount;
      tagCountPub_[i]->publish(msg);
    }
  }
}

void CalibrationComponent::callback1(ImageConstPtr const &img0) {
  std::vector<ImageMsg::ConstSharedPtr> msg_vec = {img0};
  process(msg_vec);
}

void CalibrationComponent::callback2(ImageConstPtr const &img0,
                                     ImageConstPtr const &img1) {
  std::vector<ImageMsg::ConstSharedPtr> msg_vec = {img0, img1};
  process(msg_vec);
}

void CalibrationComponent::callback3(ImageConstPtr const &img0,
                                     ImageConstPtr const &img1,
                                     ImageConstPtr const &img2) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1, img2};
  process(msg_vec);
}
void CalibrationComponent::callback4(ImageConstPtr const &img0,
                                     ImageConstPtr const &img1,
                                     ImageConstPtr const &img2,
                                     ImageConstPtr const &img3) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1, img2, img3};
  process(msg_vec);
}

void CalibrationComponent::callback5(ImageConstPtr const &img0,
                                     ImageConstPtr const &img1,
                                     ImageConstPtr const &img2,
                                     ImageConstPtr const &img3,
                                     ImageConstPtr const &img4) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1, img2, img3, img4};
  process(msg_vec);
}

void CalibrationComponent::callback6(ImageConstPtr const &img0,
                                     ImageConstPtr const &img1,
                                     ImageConstPtr const &img2,
                                     ImageConstPtr const &img3,
                                     ImageConstPtr const &img4,
                                     ImageConstPtr const &img5) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1, img2, img3, img4, img5};
  process(msg_vec);
}

}  // namespace multicam_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_calibration::CalibrationComponent)
