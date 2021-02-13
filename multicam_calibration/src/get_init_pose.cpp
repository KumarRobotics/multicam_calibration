/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "multicam_calibration/get_init_pose.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace multicam_calibration {
  namespace get_init_pose {
    // creates intrinsic matrix from 4 doubles: fx, fy, cx, cy
    cv::Mat make_intrinsic_matrix(const std::vector<double> &intr) {
      return (cv::Mat_<double>(3,3) <<
              intr[0], 0,       intr[2],
              0,       intr[1], intr[3],
              0,       0,       1.0);
    }

    void tf_to_rvec_tvec(const Eigen::Matrix<double, 4,4> &te,
                         cv::Affine3f::Vec3 *rvec, cv::Affine3f::Vec3 *tvec) {
      cv::Affine3d::Mat4 T_mat = {te(0,0), te(0,1), te(0,2), te(0,3),
                                  te(1,0), te(1,1), te(1,2), te(1,3),
                                  te(2,0), te(2,1), te(2,2), te(2,3),
                                  te(3,0), te(3,1), te(3,2), te(3,3)};
      cv::Affine3d T_cam_world(T_mat);
      *rvec = T_cam_world.rvec();
      *tvec = T_cam_world.translation();
    }


    static void decomposeHomography(const cv::Mat &HIN,
                                    cv::Mat *RR, cv::Mat *TT) {
      // H has scale invariance and scale can be negative! We can ensure that
      // x_j^T * H * x_i > 0  (Ma, top of page 136) by testing for the (z,z)
      // component of H, and flipping sign on H if it is negative.
      cv::Mat HL = (HIN.at<double>(2,2) >= 0)? HIN : -HIN;
      // first normalize the homography matrix so it has form
      //     H = R + 1/d*T*N^T
      cv::Mat SL;
      cv::SVD::compute(HL, SL);
      cv::Mat H  = HL * 1.0/SL.at<double>(0, 1);
      // now normalize and orthogonalize the first two columns,
      // which are the first two columns of the rotation matrix R
      cv::Mat r1=H.col(0)/cv::norm(H.col(0));
      cv::Mat r2 = H.col(1) - H.col(1).dot(r1)*r1;
      r2 = r2/cv::norm(r2);
      *RR = cv::Mat(3,3,CV_64F);
      r1.copyTo(RR->col(0));
      r2.copyTo(RR->col(1));
      // r3 = r1 x r2
      RR->at<double>(0,2) = r1.at<double>(1,0)*r2.at<double>(2,0) -
        r1.at<double>(2,0)*r2.at<double>(1,0);
      RR->at<double>(1,2) = r1.at<double>(2,0)*r2.at<double>(0,0) -
        r1.at<double>(0,0)*r2.at<double>(2,0);
      RR->at<double>(2,2) = r1.at<double>(0,0)*r2.at<double>(1,0) -
        r1.at<double>(1,0)*r2.at<double>(0,0);
      *TT = cv::Mat(3,1,CV_64F);
      H.col(2).copyTo(*TT);
      *TT = TT->t();
    }

    // computes a pose (rotation vector, translation) via homography
    // from world and image points. Distortion is not taken into account
    // but the projection model (i.e fisheye) is.

    Eigen::Matrix<double, 4, 4> get_init_pose(const std::vector<Point3<double>> &world_points,
                                              const std::vector<Point2<double>> &image_points,
                                              const std::vector<double> &intrinsics,
                                              const std::string &distModel,
                                              const std::vector<double> &distcoeff) {
      cv::Mat K = make_intrinsic_matrix(intrinsics);

      cv::Mat dist;
      if (distcoeff.empty()) {
        dist = (cv::Mat_<double>(4,1) << 0, 0, 0, 0);
      } else {
        dist = cv::Mat_<double>(distcoeff.size(), 1);
        for (unsigned int i = 0; i < distcoeff.size(); i++) {
          dist.at<double>(i, 0) = distcoeff[i];
        }
      }

      std::vector<cv::Point2f> wp, // world points
        ip,  // image points
        ipu; // undistorted image points

      // world points
      //std::cout << "   world points: " << std::endl;
      for (const auto &w : world_points) {
        wp.push_back(cv::Point2f(w.x, w.y));
        //std::cout << w.x << " " << w.y << std::endl;
      }
      // image points
      //std::cout << "   image points: " << std::endl;
      for (const auto &i : image_points) {
        ip.push_back(cv::Point2f(i.x, i.y));
        //std::cout << i.x << " " << i.y << std::endl;
      }
      if (distModel == "equidistant") {
        cv::fisheye::undistortPoints(ip, ipu, K, dist);
      } else if (distModel == "radtan") {
        cv::undistortPoints(ip, ipu, K, dist);
        //std::cout << "using radtan!" << std::endl;
      } else {
        std::cout << "WARNING: unknown distortion model: " << distModel << std::endl;
        ipu = ip;
      }
      //std::cout << "   image undist: " << std::endl;
      //std::cout << ipu << std::endl;
      // Use opencv to calculate the homography matrix.
      cv::Mat H = cv::findHomography(wp, ipu);

      // now decompose the homography matrix
      cv::Mat RO, TO; // optimal rotation and translation
      decomposeHomography(H, &RO, &TO);

      Eigen::Matrix<double, 4, 4> tf;
      tf(0,0) = RO.at<double>(0,0); tf(0,1) = RO.at<double>(0,1); tf(0,2) = RO.at<double>(0,2);
      tf(1,0) = RO.at<double>(1,0); tf(1,1) = RO.at<double>(1,1); tf(1,2) = RO.at<double>(1,2);
      tf(2,0) = RO.at<double>(2,0); tf(2,1) = RO.at<double>(2,1); tf(2,2) = RO.at<double>(2,2);
      tf(3,0) = 0; tf(3,1) = 0; tf(3,2) = 0; tf(3,3) = 1.0;
      
      tf(0,3) = TO.at<double>(0,0); tf(1,3) = TO.at<double>(0,1); tf(2,3) = TO.at<double>(0,2);
      return (tf);
    }

    void project_points(const std::vector<Point3<double>> &wpe,
                        const Eigen::Matrix<double, 4,4> &te,
                        const std::vector<double> &intrinsics,
                        const std::string &distModel,
                        const std::vector<double> &distcoeff,
                        std::vector<Point2<double>> *imagePoints) {
      double Kd[9] =  {intrinsics[0], 0.0, intrinsics[2],
                       0.0, intrinsics[1], intrinsics[3],
                       0.0, 0.0, 1.0};
      cv::Mat K(3, 3, CV_64FC1, Kd);
      std::vector<double> dc = distcoeff;
      while (dc.size() < 4) {
        dc.push_back(0);
      }
      cv::Mat dist(dc.size(), 1, CV_64FC1, &dc[0]);
      cv::Affine3f::Vec3 arvec, atvec;
      tf_to_rvec_tvec(te, &arvec, &atvec);
      cv::Mat im; // image points
      cv::Mat wp(wpe.size(), 1, CV_64FC3);
      for (unsigned int i = 0; i < wpe.size(); i++) {
        wp.at<cv::Vec3d>(i, 0) = cv::Vec3d(wpe[i].x, wpe[i].y, wpe[i].z);
      }
      if (distModel == "equidistant") {
        cv::fisheye::projectPoints(wp, im, arvec, atvec, K, dist);
      } else if (distModel == "radtan") {
        cv::projectPoints(wp, arvec, atvec, K, dist, im);
      } else {
        std::cout << "WARNING: unknown distortion model: " << distModel << " using radtan!" << std::endl;
        cv::projectPoints(wp, arvec, atvec, K, dist, im);
      }
      
      for (int i = 0; i < im.rows; i++) {
        imagePoints->push_back(Point2<double>(im.at<double>(i, 0), im.at<double>(i, 1)));
      }
    }
  }
}
