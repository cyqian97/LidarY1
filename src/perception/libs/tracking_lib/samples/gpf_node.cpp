/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
// For spherical voxel filtering
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "common/filters/spherical_voxel_grid.hpp"

#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/impl/transforms.hpp>
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


bool inverted_lidar_;
tf::Transform tf_rot_y;

bool use_spherical_voxel_filter;
std::vector<double> filter_leaf_sizes;

const std::string param_ns_prefix_ = "detect";  // NOLINT
std::string frame_id_;                          // NOLINT
bool use_roi_filter_;
autosense::ROIParams params_roi_;
// ROS Subscriber
ros::Subscriber pointcloud_sub_;
// ROS Publisher
ros::Publisher pcs_segmented_pub_;
/// @note Core components
boost::shared_ptr<autosense::segmenter::BaseSegmenter> ground_remover_;
boost::shared_ptr<autosense::segmenter::BaseSegmenter> segmenter_;

void OnPointCloud(const sensor_msgs::PointCloud2ConstPtr &ros_pc2) {
    autosense::common::Clock clock;

    autosense::PointICloudPtr cloud(new autosense::PointICloud);
    pcl::fromROSMsg(*ros_pc2, *cloud);
    ROS_INFO_STREAM(" Cloud inputs: #" << cloud->size() << " Points");

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    if (inverted_lidar_) {
        autosense::PointICloudPtr cloud_in(new autosense::PointICloud);
        *cloud_in = *cloud;
        cloud->clear();
        ROS_INFO_STREAM("Start to invert");
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
        voxel.setLeafSize (filter_leaf_sizes[0], filter_leaf_sizes[1], filter_leaf_sizes[2]);
        voxel.filter (*cloud);
    }

    std::vector<autosense::PointICloudPtr> cloud_clusters;
    autosense::PointICloudPtr cloud_ground(new autosense::PointICloud);
    autosense::PointICloudPtr cloud_nonground(new autosense::PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // Print the 

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_nonground, output);
    output.header = ros_pc2->header;
    pcs_segmented_pub_.publish(output);
    
    // reset clusters
    // cloud_clusters.clear();
    // segmenter_->segment(*cloud_nonground, cloud_clusters);
    // autosense::common::publishPointCloudArray<autosense::PointICloudPtr>(
    //     pcs_segmented_pub_, header, cloud_clusters);

    ROS_INFO_STREAM("Ground plane filtered. Took " << clock.takeRealTime()
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

    std::string sub_pc_topic, pub_pcs_segmented_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);

    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size",
                        sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_segmented_topic",
                        pub_pcs_segmented_topic);

    private_nh.getParam(param_ns_prefix_ + "/inverted_lidar",
                        inverted_lidar_);

    /// @note Important to use roi filter for "Ground remover"
    private_nh.param<bool>(param_ns_prefix_ + "/use_roi_filter",
                           use_roi_filter_, false);
    params_roi_ = autosense::common::getRoiParams(private_nh, param_ns_prefix_);

    private_nh.getParam(param_ns_prefix_ + "/use_spherical_voxel_filter",
                        use_spherical_voxel_filter);

    private_nh.getParam(param_ns_prefix_ + "/filter_leaf_sizes",
                        filter_leaf_sizes);
    
    ROS_INFO_STREAM("filter leaf sizes: " << filter_leaf_sizes[0] << ", " << filter_leaf_sizes[1] << ", " << filter_leaf_sizes[2]);
    
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

    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);

    // pcs_segmented_pub_ = nh.advertise<autosense_msgs::PointCloud2Array>(
    //     pub_pcs_segmented_topic, 1);

    pcs_segmented_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
        "nonground", 1);

    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        sub_pc_topic, sub_pc_queue_size, OnPointCloud);

    spiner.start();
    ROS_INFO("detection_node started...");

    ros::waitForShutdown();
    ROS_INFO("detection_node exited...");

    return 0;
}
