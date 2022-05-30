/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef COMMON_INCLUDE_COMMON_CALLI_HPP_
#define COMMON_INCLUDE_COMMON_CALLI_HPP_

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "common/types/object.hpp"  // ObjectPtr

namespace autosense {
namespace common {
namespace callibration {
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
    std::cout << "x: \n" << x << std::endl; 
    Eigen::MatrixXd m = K.inverse()*(((K*((R*x).colwise()+t)).colwise().hnormalized()).colwise().homogeneous());
    std::cout << "m: \n" << m << std::endl; 
    Eigen::MatrixXd n = m.topRows(2);
    std::cout << "n: \n" << n << std::endl; 
    std::vector<cv::Point2f> dstp;
    cv::Mat n_mat;
    cv::Mat K_mat;
    cv::eigen2cv(n,n_mat);
    cv::eigen2cv(K,K_mat);
    cv::fisheye::distortPoints(n_mat, dstp, K_mat, D_C);
    return n;
}
}  // namespace callibration
}  // namespace common
}  // namespace autosense

#endif  // COMMON_INCLUDE_COMMON_CALLIBRATION_HPP_
