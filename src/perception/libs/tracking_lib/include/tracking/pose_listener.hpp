#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>
#include <mutex>

std::mutex m;

class PoseListener {
 public:
    Eigen::Matrix4d trans;
    tf2::Transform trans_tf2;
    tf2::Transform trans_gps_lidar;

    PoseListener();

    void callbackLidarEigne(const nav_msgs::Odometry odom) {
        geometry_msgs::Pose pose;

        // pose = odom.pose.pose;

        tf2::Transform trans_odom;
        tf2::fromMsg(odom.pose.pose,trans_odom);
        m.lock();
        trans_tf2 = trans_odom * trans_gps_lidar;
        tf2::toMsg(trans_tf2, pose);

        Eigen::Affine3d trans_affine;
        Eigen::fromMsg(pose, trans_affine);
        trans = (trans_affine).matrix();
        m.unlock();
        // trans = Eigen::Matrix4d::Identity();
        // ROS_INFO_STREAM("trans: " << trans);
    }
};

PoseListener::PoseListener() {
    trans = Eigen::Matrix4d::Identity();
    tf2::Vector3 v1(4, 0, 1.5);
    tf2::Quaternion r1;
    r1.setRPY(0, 0, -3.14159 / 2);
    trans_gps_lidar = tf2::Transform(r1, v1);
};