/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 *      Kartik Mohta
 */
#include "multicam_calibration/multicam_apriltag_detector.h"

#include <cv_bridge/cv_bridge.h>
#include <boost/range/irange.hpp>
#include <iostream>
#include <sstream>

namespace multicam_calibration {
MultiCamApriltagDetector::MultiCamApriltagDetector(
    rclcpp::Node *node, const std::string &tf, int detectorType,
    int blackBorderSize, int tagRows, int tagCols, float tagSize,
    float tagSpacing, const std::string &outfname)
    : node_(node) {
  target_tag_cols_ = tagCols;
  target_tag_rows_ = tagRows;
  target_tag_size_ = tagSize;
  target_tag_spacing_ratio_ = tagSpacing;
  outfile_.open(outfname, std::ofstream::out);
  if (!outfile_) {
    throw std::runtime_error("cannot open output file " + outfname);
  } else {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "writing extracted corners to file " << outfname);
  }
  if (tf != "36h11" && tf != "25h9" && tf != "16h5") {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "invalid tag family: " << tf);
    throw std::invalid_argument("invalid tag family!");
  }
  apriltag_ros::TagFamily tagFamily = apriltag_ros::TagFamily::tf36h11;
  if (tf == "25h9") {
    tagFamily = apriltag_ros::TagFamily::tf25h9;
  } else if (tf == "16h5") {
    tagFamily = apriltag_ros::TagFamily::tf16h5;
  }

  detector_ = apriltag_ros::ApriltagDetector::Create(
      (apriltag_ros::DetectorType)detectorType, tagFamily);
  detector_->set_black_border(blackBorderSize);
}

std::vector<apriltag_ros::ApriltagVec> MultiCamApriltagDetector::process(
    std::vector<ImageConstPtr> const &msg_vec, CamWorldPoints *worldPoints,
    CamImagePoints *imagePoints) {
  std::vector<apriltag_ros::ApriltagVec> detected_tags(msg_vec.size());
  unsigned int const num_tags = target_tag_rows_ * target_tag_cols_;
  unsigned int const num_cameras = msg_vec.size();
  ++frame_count_;

  std::vector<std::vector<tag_corners_t>> tag_corners;
  tag_corners.resize(num_cameras);
  if (tag_count_.size() < num_cameras) {
    tag_count_.clear();
    for (unsigned int i = 0; i < num_cameras; i++) {
      tag_count_.push_back(0);
    }
  }
  // ros::Time t1 = ros::Time::now();
  std::vector<std::string> outstr(num_cameras);
  //#pragma omp parallel for num_threads(num_cameras)
  for (unsigned int cam_idx = 0; cam_idx < num_cameras; cam_idx++) {
    cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(
        msg_vec[cam_idx], sensor_msgs::image_encodings::MONO8);
    cv::Mat const img = cv_ptr->image;
    // detect & refine
    auto img_apriltags = detector_->Detect(img);
    // apriltag_ros::RefineApriltags(img, img_apriltags);
    detected_tags[cam_idx] = img_apriltags;
    // Store
    worldPoints->push_back(FrameWorldPoints());
    imagePoints->push_back(FrameImagePoints());
    tag_corners[cam_idx].resize(num_tags);
    for (auto const &tag : img_apriltags) {
      int const id = tag.id;
      if ((size_t)id >= num_tags) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "tag with invalid id found: "
                                << id << " (check your calibration target!)");
        continue;
      }
      std::array<Point2f, 4> const corner_positions =
          get_tag_corner_positions(id);
      tag_corners[cam_idx][id].found = true;
      for (int k = 0; k < 4; ++k) {
        const auto &wp = corner_positions[k];
        const auto &ip = tag.corners[k];
        worldPoints->back().emplace_back(wp.x, wp.y, 0);
        imagePoints->back().emplace_back(ip.x, ip.y);

        tag_corners[cam_idx][id].corners[k].x = ip.x;
        tag_corners[cam_idx][id].corners[k].y = ip.y;
        std::stringstream ss;
        ss << frame_count_ << ", intrinsics, " << cam_idx << ", " << wp.x
           << ", " << wp.y << ", " << ip.x << ", " << ip.y << std::endl;
        std::string v = ss.str();
        outstr[cam_idx] += v;
      }
      tag_count_[cam_idx]++;
    }
  }
  // must write to file outside the parallel loop to avoid garbled output
  for (const auto cam_idx : boost::irange(0u, num_cameras)) {
    outfile_ << outstr[cam_idx];
  }
  outfile_ << std::flush;

  if (frame_count_ % 10 == 0) {
    std::string ss("");
    for (auto cnt : tag_count_) {
      ss += " " + std::to_string(cnt);
    }
    RCLCPP_INFO(node_->get_logger(), "frames: %4d, total # tags found: %s",
                frame_count_, ss.c_str());
  }

  if (num_cameras < 2) return (detected_tags);

  std::vector<std::vector<Point2f>> correspondences;
  correspondences.resize(num_cameras);
  for (unsigned int tag_id = 0; tag_id < num_tags; ++tag_id) {
    bool found_in_all = true;
    for (unsigned int cam = 0; cam < num_cameras; ++cam) {
      if (!tag_corners[cam][tag_id].found) {
        found_in_all = false;
        break;
      }
    }
    if (!found_in_all) continue;

    std::array<Point2f, 4> const corner_positions =
        get_tag_corner_positions(tag_id);
    for (unsigned int i = 0; i < 4; ++i) {
      outfile_ << frame_count_ << ", extrinsics, " << corner_positions[i].x
               << ", " << corner_positions[i].y << ", ";
      for (unsigned int cam = 0; cam < num_cameras; ++cam) {
        correspondences[cam].push_back(tag_corners[cam][tag_id].corners[i]);
        outfile_ << tag_corners[cam][tag_id].corners[i].x << ", "
                 << tag_corners[cam][tag_id].corners[i].y << ", ";
      }
      outfile_ << std::endl;
    }
  }
  outfile_.flush();
  return (detected_tags);
}

std::array<MultiCamApriltagDetector::Point2f, 4>
MultiCamApriltagDetector::get_tag_corner_positions(unsigned int tag_id) const {
  unsigned int const tag_col = tag_id % target_tag_cols_;
  unsigned int const tag_row = tag_id / target_tag_cols_;

  std::array<Point2f, 4> corners;
  corners[0].x = tag_col * target_tag_size_ * (1 + target_tag_spacing_ratio_);
  corners[0].y = tag_row * target_tag_size_ * (1 + target_tag_spacing_ratio_);
  corners[1].x = corners[0].x + target_tag_size_;
  corners[1].y = corners[0].y;
  corners[2].x = corners[0].x + target_tag_size_;
  corners[2].y = corners[0].y + target_tag_size_;
  corners[3].x = corners[0].x;
  corners[3].y = corners[0].y + target_tag_size_;

  return corners;
}
}  // namespace multicam_calibration
