/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef COMMON_INCLUDE_COMMON_CALLI_HPP_
#define COMMON_INCLUDE_COMMON_CALLI_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "common/types/object.hpp"  // ObjectPtr

namespace autosense {
namespace common {
namespace calibration {
/**
 * @brief Project the lidar points to the camera 
 * @note
 * @param 
 * @param 
 * @return
 */
static Eigen::MatrixXd proj(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, std::vector<double> D_C,
                    const Eigen::MatrixXd& x) {
    // std::cout << "K: \n" << K << std::endl;
    // std::cout << "R: \n" << R << std::endl;
    // std::cout << "t: \n" << t << std::endl; 
    // std::cout << "x: \n" << x << std::endl; 

    Eigen::MatrixXd points_undistorted = (K.inverse()*(((K*((R*x).colwise()+t)).colwise().hnormalized()).colwise().homogeneous())).topRows(2);

    std::vector<cv::Point2d> points_undistorted_cv(points_undistorted.cols());
    for (int i = 0; i < points_undistorted.cols(); i++)
        points_undistorted_cv[i] = cv::Point2d(points_undistorted(1,i),points_undistorted(2,i));
        
    std::vector<cv::Point2d> points_distorted_cv;

    cv::Mat K_mat(3,3,CV_64F);
    cv::eigen2cv(K,K_mat);

    cv::Mat D_C_cv(4,1,CV_64F,D_C.data());

    

    // std::cout << "K_mat size: \n" << K_mat.size() << std::endl; 
    // std::cout << "K_mat type: \n" << K_mat.type() << std::endl; 
    // std::cout << "D_C total: \n" << D_C_cv.total() << std::endl; 

    cv::fisheye::distortPoints(points_undistorted_cv, points_distorted_cv, K_mat, D_C_cv);

    // std::cout << "Distort complete" << std::endl;

    Eigen::MatrixXd points_undistort(2,points_undistorted.cols());

    for (int i = 0; i < points_undistorted.cols(); i++)
    {
        points_undistort(1,i) = points_distorted_cv[i].x;
        points_undistort(2,i) = points_distorted_cv[i].y;
    }

    // cv::cv2eigen(dstp,r);
    
    return points_undistort;
}
}  // namespace callibration
}  // namespace common
}  // namespace autosense

#endif  // COMMON_INCLUDE_COMMON_CALLIBRATION_HPP_
