/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "multicam_calibration/test_rectify.h"
#include <apriltag_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/console.h>
#include <ros/this_node.h>
#include <string>
#include <vector>
#include <sensor_msgs/Image.h>
#include <map>

using std::string;
using std::vector;
using std::shared_ptr;

namespace multicam_calibration {
  TestRectify::TestRectify(const ros::NodeHandle& pnh) :
    nh_(pnh), imTrans_(pnh)
  {
  }

  TestRectify::~TestRectify() {
  }

  bool TestRectify::initialize() {
    imageSub_[0].reset(new Subscriber<Image>(nh_, "left/image_rect", 1));
    imageSub_[1].reset(new Subscriber<Image>(nh_, "right/image_rect", 1));
    sync_.reset(new TimeSynchronizer<Image, Image>(*imageSub_[0], *imageSub_[1], 2));
    sync_->registerCallback(boost::bind(&TestRectify::imageCallback, this, _1, _2));
    detector_ = apriltag_ros::ApriltagDetector::Create(
      apriltag_ros::DetectorType::Mit, apriltag_ros::TagFamily::tf36h11);
    detector_->set_black_border(2);
    imPub_ = imTrans_.advertise("debug_rect_image", 1);
    return (true);
  }

  static cv::Mat draw_corners(const cv::Mat &origImg,
                              std::map<int,int> idToIdx,
                              const std::vector<apriltag_ros::ApriltagVec> &tags) {
    cv::Mat img;
    origImg.copyTo(img);
    const int w = 2; // 1/2 size of plotted point
    const cv::Scalar refcol(0, 255,255);
    const cv::Scalar col(255, 0, 0);
    for (int i = 1; i < (int) tags.size(); i++) {
      for(const auto &tag : tags[i]) {
        const auto it = idToIdx.find(tag.id);
        if (it != idToIdx.end()) {
          const auto &rtag = tags[0][it->second];
          for (int k = 0; k < 4; k++) {
            cv::Point tl(rtag.corners[k].x - w, rtag.corners[k].y - w);
            cv::Point br(rtag.corners[k].x + w, rtag.corners[k].y + w);
            cv::rectangle(img, tl, br, refcol,1, 8, 0);
            cv::Point tl2(rtag.corners[k].x - w, tag.corners[k].y - w);
            cv::Point br2(rtag.corners[k].x + w, tag.corners[k].y + w);
            cv::rectangle(img, tl2, br2, col, 1, 8, 0);
          }
        }
      }
    }
    return (img);
  }


  void TestRectify::imageCallback(const ImageConstPtr &msg0, const ImageConstPtr &msg1) {
    std::vector<apriltag_ros::ApriltagVec> tags(2);
    cv::Mat img;
    std::vector<ImageConstPtr> images = {msg0, msg1};
    for (unsigned int i = 0; i < images.size(); i++) {
      const auto &imgptr = images[i];
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(imgptr,
                                      sensor_msgs::image_encodings::MONO8);
      } catch (const cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge error converting: %s", e.what());
        return;
      }
      if (i == 0) {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_GRAY2BGR);
      }
      tags[i] = detector_->Detect(cv_ptr->image);
      //apriltag_ros::RefineApriltags(cv_ptr->image, tags[i]); 
    }


    std::map<int, int> idToIdx;
    for (int j = 0; j < (int) tags[0].size(); j++) {
      idToIdx[tags[0][j].id] = j;
    }
    //
    // draw tag corners (and shifted ones from other image !)
    //
    cv::Mat dimg  = draw_corners(img, idToIdx, tags);
    //
    // calculate error in y coordinates and display
    //
    double dysum(0),dysumsq(0);
    int cnt(0);
    for (int i = 1; i < (int) images.size(); i++) {
      for(const auto &tag : tags[i]) {
        const auto it = idToIdx.find(tag.id);
        if (it != idToIdx.end()) {
          const auto &rtag = tags[0][it->second];
          for (int k = 0; k < 4; k++) {
            double dy = tag.corners[k].y - rtag.corners[k].y;
            dysum   += dy;
            dysumsq += dy * dy;
            cnt++;
          }
        }
      }
    }
    double cntinv = (cnt > 0) ? (1.0/(double)cnt) : 0.0;
    double dymean = dysum * cntinv;
    double dysig  = sqrt(fabs(cntinv * dysumsq  - dymean * dymean));
    std::string sdymean = "y mean error: " + std::to_string(dymean);
    std::string sdysig  = "y sigma:      " + std::to_string(dysig);
    cv::putText(dimg, sdymean, cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, /* fontscale */1,
                cv::Scalar(55, 000, 255), /* thickness */4);
    cv::putText(dimg, sdysig, cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, /* fontscale */1,
                cv::Scalar(55, 000, 255), /* thickness */4);
    //
    // publish image
    //
    publishImage(images[0], dimg);
  }

  void TestRectify::publishImage(const sensor_msgs::ImageConstPtr& origMsg,
                                     const cv::Mat &img) {
		if (imPub_.getNumSubscribers() > 0) {
			imPub_.publish(cv_bridge::CvImage(origMsg->header, "bgr8",
                                        img).toImageMsg());
		}
	}

}  // namespace


