/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg

#include <pcl_ros/impl/transforms.hpp> // pcl_ros::transformPointCloud, for point cloud inversion
// #include <pcl_ros/point_cloud.h> 
// #include <pcl/common/transforms.h>


// For spherical voxel filtering
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "common/filters/spherical_voxel_grid.hpp"


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"

#include "common/parameter.hpp"              // common::getSegmenterParams
#include "common/publisher.hpp"              // common::publishCloud
#include "common/time.hpp"                   // common::Clock
#include "common/types/type.h"               // PointICloudPtr
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter

#include "roi_filters/roi.hpp"  // roi::applyROIFilter

#include "nav_msgs/Odometry.h"
#include <mutex>

const std::string param_ns_prefix_ = "detect";  // NOLINT
std::string frame_id_;                          // NOLINT

bool verbose;

bool inverted_lidar_;
tf::Transform tf_rot_y;

bool use_roi_filter_;

bool use_spherical_voxel_filter;

autosense::ROIParams params_roi_;
autosense::ROIParams params_roi_second;

// ROS Subscriber
ros::Subscriber pointcloud_sub_;
ros::Subscriber nav_sub_;

// ROS Publisher
ros::Publisher pcs_segmented_pub_;
ros::Publisher pcs_non_ground_pub_;

/// @note Core components
boost::shared_ptr<autosense::segmenter::BaseSegmenter> ground_remover_;
boost::shared_ptr<autosense::segmenter::BaseSegmenter> segmenter_;



// void OnOdom(const nav_msgs::Odometry odom_)
// {

// }

void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr &ros_pc2) {
    autosense::common::Clock clock;

    autosense::PointICloudPtr cloud(new autosense::PointICloud);
    
    pcl::fromROSMsg(*ros_pc2, *cloud);
    if (verbose) ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();
    
    if (inverted_lidar_) {
        autosense::PointICloudPtr cloud_in(new autosense::PointICloud);
        *cloud_in = *cloud;
        cloud->clear();
        if (verbose) ROS_INFO_STREAM("Start to invert");
        pcl_ros::transformPointCloud(*cloud_in,*cloud,tf_rot_y);
    }

    if (use_roi_filter_) {
        autosense::roi::applyROIFilter<autosense::PointI>(params_roi_, cloud);
    }

    if (use_spherical_voxel_filter) {
        autosense::PointICloudPtr cloud_in(new autosense::PointICloud);
        *cloud_in = *cloud;
        cloud->clear();
        pcl::SphericalVoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud (cloud_in);
        voxel.setLeafSize (0.1, 180, 360);
        voxel.filter (*cloud);
    }
    std::vector<autosense::PointICloudPtr> cloud_clusters;
    autosense::PointICloudPtr cloud_ground(new autosense::PointICloud);
    autosense::PointICloudPtr cloud_nonground(new autosense::PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud_nonground, output);
    // output.header = ros_pc2->header;
    // pcs_non_ground_pub_.publish(output);

    if(params_roi_.use_second_roi_filter)
    {
        autosense::roi::applyROIFilter<autosense::PointI>(params_roi_second, cloud_nonground);
    }


    // reset clusters
    cloud_clusters.clear();
    segmenter_->segment(*cloud_nonground, cloud_clusters);
    autosense::common::publishPointCloudArray<autosense::PointICloudPtr>(
        pcs_segmented_pub_, header, cloud_clusters);

    if (verbose) ROS_INFO_STREAM("Cloud processed. Took " << clock.takeRealTime()
                                             << "ms.\n");

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detection_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    // Initialize the transform to rotate point cloud
    tf_rot_y.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 3.14159265 , 0);
    tf_rot_y.setRotation(q);


    /// @brief Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);

    std::string sub_pc_topic, pub_pcs_segmented_topic, pub_non_ground_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);

    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);

    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_segmented_topic",
                        pub_pcs_segmented_topic);

    private_nh.getParam(param_ns_prefix_ + "/pub_non_ground_topic",
                        pub_non_ground_topic);

    private_nh.getParam(param_ns_prefix_ + "/inverted_lidar",
                        inverted_lidar_);

    private_nh.getParam(param_ns_prefix_ + "/use_spherical_voxel_filter",
                        use_spherical_voxel_filter);
                        
    private_nh.getParam(param_ns_prefix_ + "/verbose", verbose);

    /// @note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter",
                           use_roi_filter_, false);

    params_roi_ = autosense::common::getRoiParams(private_nh, param_ns_prefix_);
    params_roi_second = autosense::common::getRoiParams(private_nh, param_ns_prefix_);
    if(params_roi_.use_second_roi_filter) 
    {
        params_roi_second.roi_height_above_m = params_roi_.roi_height_above_m_second;
        params_roi_second.roi_radius_max_m = params_roi_.roi_radius_max_m_second;
        params_roi_second.roi_radius_min_m = params_roi_.roi_radius_min_m_second;
    }
    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type",
                                  ground_remover_type,
                                  "GroundPlaneFittingSegmenter");
    private_nh.param<std::string>(
        param_ns_prefix_ + "/non_ground_segmenter_type",
        non_ground_segmenter_type, "RegionEuclideanSegmenter");
    autosense::SegmenterParams param =
        autosense::common::getSegmenterParams(private_nh, param_ns_prefix_);

    param.segmenter_type = ground_remover_type;
    ground_remover_ = autosense::segmenter::createGroundSegmenter(param);
    ground_remover_->verbose = verbose;

    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);
    segmenter_->verbose = verbose;

    pcs_segmented_pub_ = nh.advertise<autosense_msgs::PointCloud2Array>(
        pub_pcs_segmented_topic, 1);

    // pcs_non_ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
    //     pub_non_ground_topic, 1);

    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        sub_pc_topic, sub_pc_queue_size, OnPointCloud);

    // std::string sub_nav_topic;
    // private_nh.getParam(param_ns_prefix_ + "/sub_nav_topic",
    //                     sub_nav_topic);
    // nav_sub_ = nh.subscribe<nav_msgs::Odometry>(sub_nav_topic, 1, );


    spiner.start();
    ROS_INFO("detection_node started...");

    ros::waitForShutdown();
    ROS_INFO("detection_node exited...");

    return 0;
}
