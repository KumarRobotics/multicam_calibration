/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/calibration_test_nodelet.h"
#include "multicam_calibration/get_init_pose.h"
#include "multicam_calibration/utils.h"
#include <memory>
#include <ctime>
#include <chrono>
#include <std_msgs/UInt32.h>
#include <sstream>
#include <boost/range/irange.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

namespace multicam_calibration {
  using boost::irange;

  void CalibrationTestNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    CalibDataVec cdv = CalibrationData::parse_cameras(nh);
    nh.getParam("use_approximate_sync", useApproximateSync_);
    nh.param<bool>("use_file_extrinsics", useFileExtrinsics_, false);
    nh.param<bool>("use_file_intrinsics", useFileIntrinsics_, false);
    ROS_INFO_STREAM((useApproximateSync_ ? "" : "not ") <<  "using approximate sync");
    detector_.reset(new MultiCamApriltagDetector(nh));
    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

    image_transport::ImageTransport it(nh);
    for (const auto &camdata : cdv) {
      cameras_.push_back(std::shared_ptr<TestCamera>(new TestCamera(camdata)));
      cameras_.back()->useCameraInfo_ = !useFileIntrinsics_;
      if (cdv.size() > 1) {
        std::shared_ptr<image_transport::SubscriberFilter> p(new image_transport::SubscriberFilter());
        sub_.push_back(p);
        sub_.back()->subscribe(it, camdata.rostopic, 1);
      } else {
        singleCamSub_ = nh.subscribe(camdata.rostopic, 1,
                                     &CalibrationTestNodelet::callback1, this);
      }
      imagePub_.push_back(nh.advertise<ImageMsg>(camdata.name + "/detected_image", 1));
      std::string cname = "cam" + std::to_string(cameras_.size() - 1);
      cameras_.back()->camInfoSub_= nh.subscribe(cname +  "/camera_info", 1,
                                                 &CalibrationTestNodelet::TestCamera::camInfoCallback,
                                                 cameras_.back().get());
    }
    subscribe();
  }

  void CalibrationTestNodelet::TestCamera::camInfoCallback(const CameraInfoConstPtr &ci) {
    if (useCameraInfo_) {
      CameraIntrinsics &i = intrinsics;
      std::string dm = (ci->distortion_model == "plumb_bob") ? "radtan" : ci->distortion_model;
      i.distortion_model = dm;
      // i.camera_model     (not provided by camera_info)
      i.resolution[0] = ci->width;
      i.resolution[1] = ci->height;
    
      i.intrinsics[0] = ci->K[0];
      i.intrinsics[1] = ci->K[4];
      i.intrinsics[2] = ci->K[2];
      i.intrinsics[3] = ci->K[5];
      i.distortion_coeffs.clear();
      for (const auto &d : ci->D) {
        i.distortion_coeffs.push_back(d);
      }
      ROS_INFO_STREAM("got intrinsics: " << std::endl << i);
    } else {
      ROS_INFO_STREAM("ignoring intrinsics received from camera_info");
    }
    camInfoSub_.shutdown();
  }

  void CalibrationTestNodelet::subscribe() {
    switch (sub_.size()) {
    case 2:
      if (useApproximateSync_) {
        approx_sync2_.reset(new ApproxTimeSynchronizer2(
                              ApproxSyncPolicy2(60/*q size*/), *(sub_[0]), *(sub_[1])));
        approx_sync2_->registerCallback(&CalibrationTestNodelet::callback2, this);
      } else {
        sync2_.reset(new ExactSynchronizer2(*(sub_[0]), *(sub_[1]), 2));
        sync2_->registerCallback(&CalibrationTestNodelet::callback2, this);
      }
      break;
    case 3:
      if (useApproximateSync_) {
        approx_sync3_.reset(new ApproxTimeSynchronizer3(
                              ApproxSyncPolicy3(60/*q size*/), *(sub_[0]), *(sub_[1]), *(sub_[2])));
        approx_sync3_->registerCallback(&CalibrationTestNodelet::callback3, this);
      } else {
        sync3_.reset(new ExactSynchronizer3(*(sub_[0]), *(sub_[1]), *(sub_[2]), 2));
        sync3_->registerCallback(&CalibrationTestNodelet::callback3, this);
      }
      break;
    default:
      ROS_ERROR("invalid number of subscribers!");
    }
  }



  bool CalibrationTestNodelet::guessCameraPose(const CamWorldPoints &wp, const CamImagePoints &ip, CameraExtrinsics *T_0_w) const {
    CameraExtrinsics T_n_0 = identity();
    bool poseFound(false);
    for (unsigned int cam_idx = 0; cam_idx < cameras_.size(); cam_idx++) {
      const TestCamera &cd = *(cameras_[cam_idx]);
      T_n_0 = cd.T_cn_cnm1 * T_n_0; // chain forward from cam0 to current camera
      if (!wp[cam_idx].empty()) {
        const CameraIntrinsics &ci = cd.intrinsics;
        CameraExtrinsics T_n_w = get_init_pose::get_init_pose(wp[cam_idx], ip[cam_idx], ci.intrinsics,
                                                              ci.distortion_model, ci.distortion_coeffs);
        //std::cout << cam_idx << " init pose: " << std::endl << T_n_w << std::endl;
        if (!poseFound) {
          // first camera with valid pose found
          *T_0_w = T_n_0.inverse() * T_n_w;
          poseFound = true;
        } else {
          CameraExtrinsics T_0_w_test = T_n_0.inverse() * T_n_w;
          CameraExtrinsics T_err = T_0_w_test.inverse() * (*T_0_w);
          double rot_err = 1.5 - 0.5 *(T_err(0,0) + T_err(1,1) + T_err(2,2));
          if (rot_err > 0.25) {
            ROS_WARN_STREAM("init tf for camera " << cam_idx <<
                            " is off from your calibration by angle: " << rot_err);
            ROS_WARN_STREAM("expected T_n_0 from " << cam_idx << " to cam 0 is roughly:");
            ROS_WARN_STREAM(T_n_w * T_0_w->inverse());
          }
          //std::cout << "T_err: " << std::endl << T_err << std::endl;
          //double trans_err = T_err.block<3,1>(0,3).norm();//std::abs(T_err(3,0)) +std::abs(T_err(3,1)) +std::abs(T_err(3,2))
          //std::cout << "rot error: " << rot_err << " trans error: " << trans_err <<std::endl;
        }
      }
    }
    return (poseFound);
  }

  static bool calc_err(int cam1_idx,
                       const FrameImagePoints &ipts1,
                       const FrameWorldPoints &wpts1,
                       const CameraIntrinsics &ci1,
                       const CameraExtrinsics &T_1_w,
                       ros::Publisher &pub,
                       const ImageConstPtr &msg) {
    FrameImagePoints ipts1proj;
    get_init_pose::project_points(wpts1, T_1_w, ci1.intrinsics,
                                  ci1.distortion_model, ci1.distortion_coeffs,
                                  &ipts1proj);
    // draw points on image
    cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::MONO8);
    const cv::Mat gray = cv_ptr->image;
    if (gray.rows == 0) {
      ROS_ERROR("cannot decode image, not MONO8!");
      return (false);
    }
    cv::Mat img;
    cv::cvtColor(gray, img, CV_GRAY2BGR);
        
    double e(0);
    const cv::Scalar color(200, 0, 255);
    for (unsigned int i = 0; i < ipts1proj.size(); i++) {
      const cv::Point2d p(ipts1proj[i].x, ipts1proj[i].y);
      const int w = 2; // 1/2 size of plotted point
      cv::rectangle(img, cv::Point(p.x-w, p.y-w), cv::Point(p.x+w, p.y+w),
                    color, /*line width */ 1, /*line type*/ 8, 0);
      double dx = (ipts1[i].x - ipts1proj[i].x);
      double dy = (ipts1[i].y - ipts1proj[i].y);
      e += dx*dx + dy*dy;
    }
    cv_bridge::CvImage cv_img(msg->header, sensor_msgs::image_encodings::BGR8, img);
    pub.publish(cv_img.toImageMsg());


    double perPointErr(0);
    if (ipts1proj.size() > 0)  {
      perPointErr = std::sqrt(e / (double)ipts1proj.size());
    }
    ROS_INFO_STREAM("camera: " << cam1_idx << " points: " << ipts1proj.size() << " reproj err: " << perPointErr);
    return (true);
  }
  
  
  void CalibrationTestNodelet::process(const std::vector<ImageConstPtr> &msg_vec) {
    CamWorldPoints wp;
    CamImagePoints ip;
    std::vector<apriltag_ros::ApriltagVec> detected_tags =
      detector_->process(msg_vec, &wp, &ip);
    ROS_ASSERT(msg_vec.size() == detected_tags.size());
    // Insert newly detected points into existing set.
    // At this point one could add the data points to an
    // incremental solver.
    CameraExtrinsics cam0Pose;
    if (!guessCameraPose(wp, ip, &cam0Pose)) {
      ROS_WARN("no detections found, skipping frame!");
      return;
    }
    
    CameraExtrinsics T_0_w = cam0Pose;
    CameraExtrinsics T_1_0 = identity();
    
    struct CamErr {
      double errSum {0};
      int    cnt {0};
    };
    
    std::vector<CamErr> cam_error(cameras_.size());
    std::string cam0_frame = msg_vec[0]->header.frame_id;
    for (unsigned int cam1_idx = 0; cam1_idx < cameras_.size(); cam1_idx++) {
      const TestCamera cam1 = *cameras_[cam1_idx];
      if (useFileExtrinsics_) {
        T_1_0 = cam1.T_cn_cnm1 * T_1_0;  // forward to next camera;
      } else {
        std::string cam1_frame = msg_vec[cam1_idx]->header.frame_id;
        std::cout << "looking up: " << cam0_frame << " -> " << cam1_frame << std::endl;
        try {
          geometry_msgs::TransformStamped tf_1_0 = tfBuffer_.lookupTransform(
            cam1_frame, cam0_frame, ros::Time(0));
          Eigen::Isometry3d tf_1_0_eigen;
          tf::transformMsgToEigen(tf_1_0.transform, tf_1_0_eigen);
          T_1_0 = tf_1_0_eigen.matrix();
        } catch(tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          return;
        }
      }
      std::cout << "using ext: " << std::endl << T_1_0 << std::endl;
      const CameraExtrinsics T_1_w = T_1_0 * T_0_w;
      const auto &wpts1 = wp[cam1_idx];
      if (!wpts1.empty()) {
#if 1
        if (!calc_err(cam1_idx,
                      ip[cam1_idx],
                      wpts1,
                      cam1.intrinsics,
                      T_1_w,
                      imagePub_[cam1_idx],
                      msg_vec[cam1_idx])) {
          continue;
        }
#else        
        const auto &ipts1 = 
        const CameraIntrinsics &ci1 = cam1.intrinsics;
        FrameImagePoints ipts1proj;
        get_init_pose::project_points(wpts1, T_1_w, ci1.intrinsics,
                                      ci1.distortion_model, ci1.distortion_coeffs,
                                      &ipts1proj);
        // draw points on image
        cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(
          msg_vec[cam1_idx], sensor_msgs::image_encodings::MONO8);
        const cv::Mat gray = cv_ptr->image;
        if (gray.rows == 0) {
          ROS_ERROR("cannot decode image, not MONO8!");
          continue;
        }
        cv::Mat img;
        cv::cvtColor(gray, img, CV_GRAY2BGR);
        
        double e(0);
        const cv::Scalar color(200, 0, 255);
        for (unsigned int i = 0; i < ipts1.size(); i++) {
          const cv::Point2d p(ipts1proj[i].x, ipts1proj[i].y);
          const int w = 2; // 1/2 size of plotted point
          cv::rectangle(img, cv::Point(p.x-w, p.y-w), cv::Point(p.x+w, p.y+w),
                        color, /*line width */ 1, /*line type*/ 8, 0);
          double dx = (ipts1[i].x - ipts1proj[i].x);
          double dy = (ipts1[i].y - ipts1proj[i].y);
          e += dx*dx + dy*dy;
        }
        cv_bridge::CvImage cv_img(msg_vec[cam1_idx]->header, sensor_msgs::image_encodings::BGR8, img);
        imagePub_[cam1_idx].publish(cv_img.toImageMsg());


        CamErr &ce = cam_error[cam1_idx]; // per-camera error
        ce.errSum += e;
        ce.cnt    += ipts1.size();
        double perPointErr(0);
        if (ce.cnt >= 0)  {
          perPointErr = std::sqrt(ce.errSum / (double)ce.cnt);
        }
        ROS_INFO_STREAM("camera: " << cam1_idx << " points: " << ce.cnt << " reproj err: " << perPointErr);
#endif        
      } else {
        ROS_WARN_STREAM("skipping camera " << cam1_idx << " (no points!)");
        continue;
      }
    }
  }

  void CalibrationTestNodelet::publishDebugImages(const std::vector<ImageMsg::ConstPtr> &msg_vec,
                                                  const std::vector<apriltag_ros::ApriltagVec> &detected_tags) {
    for (int i = 0; i < (int)detected_tags.size(); i++) {
      if (imagePub_[i].getNumSubscribers() > 0) {
        cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(
          msg_vec[i], sensor_msgs::image_encodings::MONO8);
        const cv::Mat gray = cv_ptr->image;
        if (gray.rows == 0) {
          ROS_ERROR("cannot decode image, not MONO8!");
          continue;
        }
        cv::Mat img;
        cv::cvtColor(gray, img, CV_GRAY2BGR);
        apriltag_ros::DrawApriltags(img, detected_tags[i]);
        cv_bridge::CvImage cv_img(msg_vec[i]->header, sensor_msgs::image_encodings::BGR8, img);
        imagePub_[i].publish(cv_img.toImageMsg());
      }
    }
  }
  
  void CalibrationTestNodelet::callback1(ImageConstPtr const &img0) {
    std::vector<ImageMsg::ConstPtr> msg_vec = {img0};
    process(msg_vec);
  }
  
  void CalibrationTestNodelet::callback2(ImageConstPtr const &img0, ImageConstPtr const &img1) {
    std::vector<ImageMsg::ConstPtr> msg_vec = {img0, img1};
    process(msg_vec);
  }

  void CalibrationTestNodelet::callback3(ImageConstPtr const &img0, ImageConstPtr const &img1,
                                     ImageConstPtr const &img2) {
    std::vector<ImageMsg::ConstPtr> msg_vec = {img0, img1, img2};
    process(msg_vec);
  }
}
