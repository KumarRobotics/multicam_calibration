/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibration_test_component.h"
#include "multicam_calibration/get_init_pose.h"
#include "multicam_calibration/utils.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt32.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <boost/range/irange.hpp>

#include <chrono>
#include <ctime>
#include <memory>
#include <sstream>

namespace multicam_calibration {
using boost::irange;

CalibrationTestComponent::CalibrationTestComponent(
    const rclcpp::NodeOptions &options)
    : Node("calibration", options) {
  CameraExtrinsics T_imu_body;
  const std::string calibFile =
      declare_parameter("calibration_file", "calib.yaml");
  CalibDataVec cdv = CalibrationData::parse_cameras(calibFile, &T_imu_body);
  useApproximateSync_ = declare_parameter("use_approximate_sync", true);
  useFileExtrinsics_ = declare_parameter("use_file_extrinsics", false);
  useFileIntrinsics_ = declare_parameter("use_file_intrinsics", false);
  RCLCPP_INFO_STREAM(get_logger(), (useApproximateSync_ ? "" : "not ")
                                       << "using approximate sync");
  const std::string cornersOutFile =
      declare_parameter("corners_out_file", "corners.csv");
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
  rclcpp::Clock::SharedPtr clock = get_clock();
  tfBuffer_.reset(new tf2_ros::Buffer(clock));
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));

  for (const auto &camdata : cdv) {
    const auto qos = rmw_qos_profile_default;

    cameras_.push_back(std::shared_ptr<TestCamera>(new TestCamera(camdata)));
    cameras_.back()->useCameraInfo_ = !useFileIntrinsics_;
    if (cdv.size() > 1) {
      sub_.push_back(std::make_shared<message_filters::Subscriber<ImageMsg>>(
          this, camdata.rostopic, qos));
    } else {
      singleCamSub_ = image_transport::create_subscription(
          this, camdata.rostopic,
          std::bind(&CalibrationTestComponent::callback1, this,
                    std::placeholders::_1),
          "raw", qos);
    }
    imagePub_.push_back(image_transport::create_publisher(
        this, camdata.name + "/detected_image", qos));
    std::string cname = "cam" + std::to_string(cameras_.size() - 1);
    cameras_.back()->camInfoSub_ = create_subscription<CameraInfo>(
        cname + "/camera_info", 10,
        std::bind(&CalibrationTestComponent::TestCamera::camInfoCallback,
                  cameras_.back().get(), std::placeholders::_1));
  }
  setupSync();
}

void CalibrationTestComponent::TestCamera::camInfoCallback(
    CameraInfoConstPtr ci) {
  if (useCameraInfo_) {
    CameraIntrinsics &i = intrinsics;
    std::string dm =
        (ci->distortion_model == "plumb_bob") ? "radtan" : ci->distortion_model;
    i.distortion_model = dm;
    // i.camera_model     (not provided by camera_info)
    i.resolution[0] = ci->width;
    i.resolution[1] = ci->height;

    i.intrinsics[0] = ci->k[0];
    i.intrinsics[1] = ci->k[4];
    i.intrinsics[2] = ci->k[2];
    i.intrinsics[3] = ci->k[5];
    i.distortion_coeffs.clear();
    for (const auto &d : ci->d) {
      i.distortion_coeffs.push_back(d);
    }
    std::cout << "got intrinsics: " << std::endl << i << std::endl;
  } else {
    std::cout << "ignoring intrinsics received from camera_info" << std::endl;
  }
  camInfoSub_.reset();
}

void CalibrationTestComponent::setupSync() {
  switch (sub_.size()) {
    case 0:
      // no subscribers, must be single camera
      return;
    case 2:
      if (useApproximateSync_) {
        approx_sync2_.reset(new ApproxTimeSynchronizer2(
            ApproxSyncPolicy2(60 /*q size*/), *(sub_[0]), *(sub_[1])));
        approx_sync2_->registerCallback(&CalibrationTestComponent::callback2,
                                        this);
      } else {
        sync2_.reset(new ExactSynchronizer2(*(sub_[0]), *(sub_[1]), 2));
        sync2_->registerCallback(&CalibrationTestComponent::callback2, this);
      }
      break;
    case 3:
      if (useApproximateSync_) {
        approx_sync3_.reset(
            new ApproxTimeSynchronizer3(ApproxSyncPolicy3(60 /*q size*/),
                                        *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approx_sync3_->registerCallback(&CalibrationTestComponent::callback3,
                                        this);
      } else {
        sync3_.reset(
            new ExactSynchronizer3(*(sub_[0]), *(sub_[1]), *(sub_[2]), 2));
        sync3_->registerCallback(&CalibrationTestComponent::callback3, this);
      }
      break;
    default:
      RCLCPP_ERROR_STREAM(get_logger(),
                          "invalid number of subscribers: " << sub_.size());
  }
}

bool CalibrationTestComponent::guessCameraPose(const CamWorldPoints &wp,
                                               const CamImagePoints &ip,
                                               CameraExtrinsics *T_0_w) const {
  CameraExtrinsics T_n_0 = identity();
  bool poseFound(false);
  for (unsigned int cam_idx = 0; cam_idx < cameras_.size(); cam_idx++) {
    const TestCamera &cd = *(cameras_[cam_idx]);
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
          RCLCPP_WARN_STREAM(
              get_logger(),
              "init tf for camera "
                  << cam_idx
                  << " is off from your calibration by angle: " << rot_err);
          RCLCPP_WARN_STREAM(get_logger(), "expected T_n_0 from "
                                               << cam_idx
                                               << " to cam 0 is roughly:");
          RCLCPP_WARN_STREAM(get_logger(), T_n_w * T_0_w->inverse());
        }
        // std::cout << "T_err: " << std::endl << T_err << std::endl;
        // double trans_err =
        // T_err.block<3,1>(0,3).norm();//std::abs(T_err(3,0))
        // +std::abs(T_err(3,1)) +std::abs(T_err(3,2)) std::cout << "rot error:
        // "
        // << rot_err << " trans error: " << trans_err <<std::endl;
      }
    }
  }
  return (poseFound);
}

static bool calc_err(int cam1_idx, const FrameImagePoints &ipts1,
                     const FrameWorldPoints &wpts1, const CameraIntrinsics &ci1,
                     const CameraExtrinsics &T_1_w,
                     image_transport::Publisher &pub,
                     const ImageConstPtr &msg) {
  FrameImagePoints ipts1proj;
  get_init_pose::project_points(wpts1, T_1_w, ci1.intrinsics,
                                ci1.distortion_model, ci1.distortion_coeffs,
                                &ipts1proj);
  // draw points on image
  cv_bridge::CvImageConstPtr const cv_ptr =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  const cv::Mat gray = cv_ptr->image;
  if (gray.rows == 0) {
    std::cerr << "cannot decode image, not MONO8!" << std::endl;
    return (false);
  }
  cv::Mat img;
  cv::cvtColor(gray, img, CV_GRAY2BGR);

  double e(0);
  const cv::Scalar color_proj(200, 0, 255);
  const cv::Scalar color_orig(200, 255, 0);
  for (unsigned int i = 0; i < ipts1proj.size(); i++) {
    const cv::Point2d p(ipts1proj[i].x, ipts1proj[i].y);
    const int w = 2;  // 1/2 size of plotted point
    cv::rectangle(img, cv::Point(p.x - w, p.y - w), cv::Point(p.x + w, p.y + w),
                  color_proj, /*line width */ 1, /*line type*/ 8, 0);
    cv::rectangle(img, cv::Point(ipts1[i].x - w, ipts1[i].y - w),
                  cv::Point(ipts1[i].x + w, ipts1[i].y + w), color_orig,
                  /*line width */ 1, /*line type*/ 8, 0);
    double dx = (ipts1[i].x - ipts1proj[i].x);
    double dy = (ipts1[i].y - ipts1proj[i].y);
    e += dx * dx + dy * dy;
  }
  cv_bridge::CvImage cv_img(msg->header, sensor_msgs::image_encodings::BGR8,
                            img);
  pub.publish(cv_img.toImageMsg());

  double perPointErr(0);
  if (ipts1proj.size() > 0) {
    perPointErr = std::sqrt(e / (double)ipts1proj.size());
  }
  std::cout << "camera: " << cam1_idx << " points: " << ipts1proj.size()
            << " reproj err: " << perPointErr << std::endl;
  return (true);
}

void CalibrationTestComponent::process(
    const std::vector<ImageConstPtr> &msg_vec) {
  CamWorldPoints wp;
  CamImagePoints ip;
  std::vector<apriltag_ros::ApriltagVec> detected_tags =
      detector_->process(msg_vec, &wp, &ip);
  assert(msg_vec.size() == detected_tags.size());
  // Insert newly detected points into existing set.
  // At this point one could add the data points to an
  // incremental solver.
  CameraExtrinsics cam0Pose;
  if (!guessCameraPose(wp, ip, &cam0Pose)) {
    RCLCPP_WARN(get_logger(), "no detections found, skipping frame!");
    return;
  }

  CameraExtrinsics T_0_w = cam0Pose;
  CameraExtrinsics T_1_0 = identity();

  struct CamErr {
    double errSum{0};
    int cnt{0};
  };

  std::vector<CamErr> cam_error(cameras_.size());
  std::string cam0_frame = msg_vec[0]->header.frame_id;
  for (unsigned int cam1_idx = 0; cam1_idx < cameras_.size(); cam1_idx++) {
    const TestCamera cam1 = *cameras_[cam1_idx];
    if (useFileExtrinsics_) {
      T_1_0 = cam1.T_cn_cnm1 * T_1_0;  // forward to next camera;
    } else {
      std::string cam1_frame = msg_vec[cam1_idx]->header.frame_id;
      std::cout << "looking up: " << cam0_frame << " -> " << cam1_frame
                << std::endl;
      try {
        geometry_msgs::msg::TransformStamped tf_1_0 =
            tfBuffer_->lookupTransform(cam1_frame, cam0_frame,
                                       tf2::TimePoint());
        Eigen::Isometry3d tf_1_0_eigen = tf2::transformToEigen(tf_1_0);
        T_1_0 = tf_1_0_eigen.matrix();
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        return;
      }
    }
    std::cout << "using ext: " << std::endl << T_1_0 << std::endl;
    const CameraExtrinsics T_1_w = T_1_0 * T_0_w;
    const auto &wpts1 = wp[cam1_idx];
    if (!wpts1.empty()) {
      if (!calc_err(cam1_idx, ip[cam1_idx], wpts1, cam1.intrinsics, T_1_w,
                    imagePub_[cam1_idx], msg_vec[cam1_idx])) {
        continue;
      }
    } else {
      RCLCPP_WARN_STREAM(get_logger(),
                         "skipping camera " << cam1_idx << " (no points!)");
      continue;
    }
  }
}

void CalibrationTestComponent::publishDebugImages(
    const std::vector<ImageConstPtr> &msg_vec,
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
      cv_bridge::CvImage cv_img(msg_vec[i]->header,
                                sensor_msgs::image_encodings::BGR8, img);
      imagePub_[i].publish(cv_img.toImageMsg());
    }
  }
}

void CalibrationTestComponent::callback1(ImageConstPtr const &img0) {
  std::vector<ImageConstPtr> msg_vec = {img0};
  process(msg_vec);
}

void CalibrationTestComponent::callback2(ImageConstPtr const &img0,
                                         ImageConstPtr const &img1) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1};
  process(msg_vec);
}

void CalibrationTestComponent::callback3(ImageConstPtr const &img0,
                                         ImageConstPtr const &img1,
                                         ImageConstPtr const &img2) {
  std::vector<ImageConstPtr> msg_vec = {img0, img1, img2};
  process(msg_vec);
}
}  // namespace multicam_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_calibration::CalibrationTestComponent)
