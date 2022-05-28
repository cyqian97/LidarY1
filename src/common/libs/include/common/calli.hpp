// /*
//  * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
//  * Gary Chan <chenshj35@mail2.sysu.edu.cn>
//  */

// #ifndef COMMON_INCLUDE_COMMON_CALLI_HPP_
// #define COMMON_INCLUDE_COMMON_CALLI_HPP_

// #include <pcl/point_cloud.h>
// #include <Eigen/Core>

// #include "common/types/object.hpp"  // ObjectPtr

// namespace autosense {
// namespace common {
// namespace callibration {
// /**
//  * @brief Project the lidar points to the camera 
//  * @note
//  * @param 
//  * @param 
//  * @return
//  */
// static Eigen::MatrixXd proj(const Eigen::MatrixXd& K, const Eigen::MatrixXd& R, const Eigen::MatrixXd& t,
//                     const Eigen::MatrixXf& x) {
//     Eigen::MatrixXd m = K*(R*x+t)
//     return m
// }
// }  // namespace callibration
// }  // namespace common
// }  // namespace autosense

// #endif  // COMMON_INCLUDE_COMMON_CALLIBRATION_HPP_