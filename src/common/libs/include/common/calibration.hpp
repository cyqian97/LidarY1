/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef COMMON_INCLUDE_COMMON_CALI_HPP_
#define COMMON_INCLUDE_COMMON_CALI_HPP_

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
                    const Eigen::MatrixXd& x, const bool invert_lidar =  true) {//const Eigen::MatrixXd& x)

    // Convert Eigne points to cv points
    std::vector<cv::Point3f> points_lidar(x.cols());
    for (int i = 0; i < x.cols(); i++)
        points_lidar[i] = cv::Point3f(x(0,i),x(1,i),x(2,i));
    // Project the points
    std::vector<cv::Point2f> points_distorted_cv;

    cv::Mat R_mat(3,3,CV_64F);
    Eigen::Matrix3d RR = R.inverse();
    cv::eigen2cv(RR,R_mat);
    
    cv::Mat t_mat(3,1,CV_64F);
    Eigen::Vector3d tt = -t;
    cv::eigen2cv(tt,t_mat);

    cv::Mat K_mat(3,3,CV_64F);
    cv::eigen2cv(K,K_mat);

    cv::Mat D_C_cv(5,1,CV_64F,D_C.data());


    cv::projectPoints(points_lidar, R_mat, t_mat, K_mat, D_C_cv, points_distorted_cv);
    

    // std::cout << "Distort complete" << std::endl;

    
    Eigen::MatrixXd points_distorted(2,x.cols());

    for (int i = 0; i < x.cols(); i++)
    {
        points_distorted(0,i) = points_distorted_cv[i].x;
        points_distorted(1,i) = points_distorted_cv[i].y;
        // if (i%20 == 0)
        //     std::cout << "Point: " <<  points_distorted(0,i) << ", " <<  points_distorted(0,i) << std::endl;
    }


    // std::cout << "points_distorted: \n" << points_distorted << std::endl; 
    // cv::cv2eigen(dstp,r);
    
    return points_distorted;
}

static Eigen::MatrixXd projFisheye(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, std::vector<double> D_C,
                    const Eigen::MatrixXd& x, const bool invert_lidar =  true) {//const Eigen::MatrixXd& x)

    // Lidar is mounted inverted.
    Eigen::Matrix3d rot_y;
    rot_y <<    -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, -1.0;


    // std::cout << "K: \n" << K << std::endl;
    // std::cout << "R: \n" << R << std::endl;
    // std::cout << "t: \n" << t << std::endl; 
    // std::cout << "D: \n";

    // for(const auto& d: D_C)
    //     std::cout << d << "\t";
    // std::cout << std::endl;

    // std::cout << "x: \n" << x << std::endl; 
    Eigen::MatrixXd points_undistorted;
    if(invert_lidar)
    {
        points_undistorted = (K.inverse()*(((K*((R*rot_y*x).colwise()+t)).colwise().hnormalized()).colwise().homogeneous())).topRows(2);
    }
    else
    {
        points_undistorted = (K.inverse()*(((K*((R*x).colwise()+t)).colwise().hnormalized()).colwise().homogeneous())).topRows(2);
    }

    std::vector<cv::Point2d> points_undistorted_cv(points_undistorted.cols());
    for (int i = 0; i < points_undistorted.cols(); i++)
        points_undistorted_cv[i] = cv::Point2d(points_undistorted(0,i),points_undistorted(1,i));

    
    // std::cout << "points_undistorted: \n" << (((K*((R*x).colwise()+t)).colwise().hnormalized()).colwise().homogeneous()) << std::endl; 
    // std::cout << "points_undistorted : \n" << points_undistorted << std::endl; 
    // std::cout << "points_undistorted_cv: \n" << points_undistorted_cv << std::endl; 
        
    std::vector<cv::Point2d> points_distorted_cv;

    cv::Mat K_mat(3,3,CV_64F);
    cv::eigen2cv(K,K_mat);

    cv::Mat D_C_cv(4,1,CV_64F,D_C.data());

    // std::cout << "K_mat: \n" << K_mat << std::endl;
    // std::cout << "D_C_cv: \n" << D_C_cv << std::endl;

    

    // std::cout << "K_mat size: \n" << K_mat.size() << std::endl; 
    // std::cout << "K_mat type: \n" << K_mat.type() << std::endl; 
    // std::cout << "D_C total: \n" << D_C_cv.total() << std::endl; 

    cv::fisheye::distortPoints(points_undistorted_cv, points_distorted_cv, K_mat, D_C_cv);

    // std::cout << "Distort complete" << std::endl;

    Eigen::MatrixXd points_distorted(2,points_undistorted.cols());

    for (int i = 0; i < points_undistorted.cols(); i++)
    {
        points_distorted(0,i) = points_distorted_cv[i].x;
        points_distorted(1,i) = points_distorted_cv[i].y;
    }


    // std::cout << "points_distorted: \n" << points_distorted << std::endl; 
    // cv::cv2eigen(dstp,r);
    
    return points_distorted;
}

}  // namespace calibration
}  // namespace common
}  // namespace autosense

#endif  // COMMON_INCLUDE_COMMON_CALIBRATION_HPP_
