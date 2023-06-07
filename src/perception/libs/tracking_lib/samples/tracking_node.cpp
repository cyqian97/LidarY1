/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <memory>
#include <pcl_ros/impl/transforms.hpp>

// #include "opencv2/opencv.hpp"
// #include "opencv2/core/eigen.hpp"
// #include "opencv2/calib3d.hpp"
// #include "opencv2/core/mat.hpp"

// gps message type
#include <nav_msgs/Odometry.h>

// bounding box message type for local planner
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/BoundingBox3DArray.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "classifiers/classifier_manager.hpp"
#include "common/bounding_box.hpp"
#include "common/calibration.hpp"
#include "common/color.hpp"
#include "common/id_pub_manager.hpp"
#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"
#include "common/parameter.hpp"
#include "common/publisher.hpp"
#include "common/time.hpp"
#include "common/transform.hpp"
#include "common/types/object.hpp"
#include "common/types/type.h"
#include "object_builders/object_builder_manager.hpp"
#include "perception_msgs/Object.h"
#include "perception_msgs/Objects.h"
#include "perception_msgs/pos3d.h"
#include "tracking/tracking_worker_manager.hpp"

bool verbose;
bool visualize;

nav_msgs::Odometry odom;
ros::Subscriber nav_sub_;
ros::Publisher bbox_pub_;
void OnNavOdom(const nav_msgs::Odometry odom_) {
    odom = odom_;
    // ROS_INFO_STREAM(" UTM: " << odom.pose.pose.position.x << ", " <<
    // odom.pose.pose.position.y << ", " << odom.pose.pose.position.z);
};

// std::map<autosense::IdType, std::vector<autosense::ObjectType>>
// type_histories; std::map<autosense::IdType, autosense::ObjectType>
// type_fixed; const std::vector<autosense::ObjectType>
// type_car_vector(5,autosense::CAR); const std::vector<autosense::ObjectType>
// type_ped_vector(5,autosense::PEDESTRIAN);

const std::string param_ns_prefix_ = "tracking";  // NOLINT
std::string local_frame_id_, global_frame_id_;    // NOLINT
int tf_timeout_ms_ = 0;
double threshold_contian_IoU_ = 0.0;
double pub_course_speed_limit;
std::vector<double> offset(3, 0.0);
bool care_object_only;

autosense::TrackingWorkerParams tracking_params_;
autosense::ClassifierParams classifier_params_;

// ROS service
//  ros::ServiceServer srv_pos3d;
//  std::shared_ptr<std::vector<autosense::PointICloudPtr>> non_ground_copy =
//  nullptr;

// For projection demonstration and service
Eigen::Matrix3d K_C;
Eigen::Matrix3d R_Lidar_CameraC;
Eigen::Vector3d t_Lidar_CameraC;
std::vector<double> D_C(4, 0.);
std::shared_ptr<std::vector<autosense::PointICloudPtr>> non_ground_copy =
    nullptr;

// ROS Subscriber
ros::Subscriber pcs_segmented_sub_;
std::unique_ptr<tf::TransformListener> tf_listener_;

ros::Subscriber gps_sub_;
double theta = 0.0;

// ros::Subscriber image_sub_;
// cv_bridge::CvImagePtr cv_ptr;

// ROS Publisher
ros::Publisher segments_coarse_pub_;
ros::Publisher segments_predict_pub_;
ros::Publisher segments_pub_;
ros::Publisher tracking_output_objects_pub_;
ros::Publisher tracking_output_trajectories_pub_;
ros::Publisher tracking_objects_pub_;
ros::Publisher tracking_objects_cloud_pub_;
ros::Publisher tracking_objects_velocity_pub_;
ros::Publisher tracking_objects_tracker_id_pub_;
ros::Publisher tracking_objects_trajectory_pub_;
ros::Publisher
    lidar_camera_pub_;  // Publish the final message for canbus scoring

autosense::common::IdPubManager<autosense::IdPubType> *id_pub_publisher_ =
    nullptr;

// For projection demonstration only
ros::Publisher pcs_distort_pub_;
// tf::Transform tf_rot_y;

/// @note Core components
std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ =
    nullptr;
std::unique_ptr<autosense::tracking::BaseTrackingWorker> tracking_worker_ =
    nullptr;
std::unique_ptr<autosense::classifier::BaseClassifier> classifier_worker_ =
    nullptr;

std_msgs::Header hd;
// TODO(chenshengjie): callback function as fast as possible
void OnSegmentClouds(
    const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg) {
    const double kTimeStamp = segments_msg->header.stamp.toSec();
    if (verbose)
        ROS_INFO("Tracking: Clusters size: %d at %lf.",
                 segments_msg->clouds.size(), kTimeStamp);
    hd = segments_msg->header;
    std_msgs::Header header;
    header.frame_id = local_frame_id_;
    header.stamp = segments_msg->header.stamp;

    // initial coarse segments directly from segment node or after classified by
    // learning node
    std::vector<autosense::PointICloudPtr> segment_clouds;
    for (size_t i = 0u; i < segments_msg->clouds.size(); ++i) {
        autosense::PointICloudPtr cloud(new autosense::PointICloud);
        pcl::fromROSMsg(segments_msg->clouds[i], *cloud);
        segment_clouds.push_back(cloud);
    }

    // For projection demonstration only
    non_ground_copy = std::make_shared<std::vector<autosense::PointICloudPtr>>(
        segment_clouds);
    if (nullptr != non_ground_copy) {
        if (verbose)
            ROS_INFO_STREAM("copied cloud size: " << non_ground_copy->size());

        autosense::PointICloudPtr cloud_combined_I(new autosense::PointICloud);
        autosense::PointCloudPtr cloud_combined(new autosense::PointCloud);

        for (const auto &cloud : *non_ground_copy) *cloud_combined_I += *cloud;
        pcl::copyPointCloud(*cloud_combined_I, *cloud_combined);

        // autosense::PointCloudPtr cloud_in(new autosense::PointCloud);
        // *cloud_in = *cloud_combined;
        // cloud_combined->clear();
        // pcl_ros::transformPointCloud(*cloud_in,*cloud_combined,tf_rot_y);

        auto m = cloud_combined->getMatrixXfMap(3, 4, 0);

        // auto m2 = m.topLeftCorner(3,2);
        // Eigen::MatrixXd x = m2.cast <double> ();
        Eigen::MatrixXd x = m.cast<double>();

        // ROS_INFO_STREAM("\t mat cols: " << m2.cols());
        // ROS_INFO_STREAM("\t mat rows: " << m2.rows());
        // Eigen::MatrixXd x = m2.cast <double> ();
        // if(verbose)
        // {
        //     ROS_INFO_STREAM("\t mat cols: " << m2.cols());
        //     ROS_INFO_STREAM("\t mat rows: " << m2.rows());
        // }

        Eigen::MatrixXd res = autosense::common::calibration::proj(
            K_C, R_Lidar_CameraC, t_Lidar_CameraC, D_C, x);
        autosense::PointCloudPtr cloud_distort(new autosense::PointCloud);

        for (int i = 0; i < res.cols(); ++i) {
            autosense::Point p(double(res(0, i)), double(res(1, i)), 0.0);
            cloud_distort->points.push_back(p);
        }

        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(*cloud_distort, output);

        output.header = hd;
        pcs_distort_pub_.publish(output);
    }

    // current pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    auto status = autosense::common::transform::getVelodynePose(
        *tf_listener_, local_frame_id_, global_frame_id_, kTimeStamp, &pose);
    if (!status) {
        ROS_WARN("Failed to fetch current pose, tracking skipped...");
        return;
    }
    auto velo2world = std::make_shared<Eigen::Matrix4d>(pose);

    // object builder
    autosense::common::Clock clock_builder;
    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(segment_clouds, &objects);
    if (verbose)
        ROS_INFO_STREAM("Objects built. Took " << clock_builder.takeRealTime()
                                               << "ms.");

    // visualize initial coarse segments
    autosense::common::publishObjectsMarkers(
        segments_coarse_pub_, header, autosense::common::MAGENTA.rgbA, objects);

    /**
     * @brief Use Tracking temporal information to improve segmentation
     * @note
     *  <1> project coarse segmentation into World coordinate
     *  <2> get expected objectd (in world coordinate)
     *  <3> over-segmentation/under-segmentation detect and improve segmentation
     *  <4> project back to current Velodyne coordinate
     */
    std::vector<autosense::ObjectPtr> expected_objects =
        tracking_worker_->collectExpectedObjects(kTimeStamp, *velo2world);
    std::vector<autosense::ObjectPtr> obsv_objects(objects.begin(),
                                                   objects.end());
    // /// @note Tracking 就靠粗分割进行初始化
    // if (!expected_objects.empty()) {
    //     /// @note 通过Tracking信息提升分割：过/欠分割,
    //     /// 主要目的是因为遮挡等造成的过分割
    //     // TODO(chenshengjie): 能否结合一些历史做一下当前完整修饰
    //     for (size_t expected_idx = 0u; expected_idx <
    //     expected_objects.size();
    //          ++expected_idx) {
    //         autosense::common::bbox::GroundBox gbox_expected;
    //         autosense::common::bbox::toGroundBox(expected_objects[expected_idx],
    //                                              &gbox_expected);

    //         autosense::ObjectPtr object_merged(new autosense::Object);

    //         for (size_t obsv_idx = 0u; obsv_idx < objects.size(); ++obsv_idx)
    //         {
    //             autosense::common::bbox::GroundBox gbox_obsv;
    //             autosense::common::bbox::toGroundBox(objects[obsv_idx],
    //                                                  &gbox_obsv);

    //             // combining all connected components within an expected
    //             // object’s bounding box into a new one
    //             if (autosense::common::bbox::groundBoxOverlap(
    //                     gbox_expected, gbox_obsv, threshold_contian_IoU_)) {
    //                 *object_merged->cloud += *objects[obsv_idx]->cloud;
    //                 obsv_objects[obsv_idx]->cloud->clear();
    //             }
    //         }
    //         // build merged object
    //         object_builder_->build(object_merged);
    //         // maintain tracking-help segmented objects
    //         obsv_objects.push_back(object_merged);
    //     }
    //     // remove all connected components at once
    //     auto iter = obsv_objects.begin();
    //     for (; iter != obsv_objects.end();) {
    //         if ((*iter)->cloud->empty()) {
    //             iter = obsv_objects.erase(iter);
    //         } else {
    //             ++iter;
    //         }
    //     }
    // }

    // visualize expected objects
    autosense::common::publishObjectsMarkers(segments_predict_pub_, header,
                                             autosense::common::DARKGREEN.rgbA,
                                             expected_objects);
    // visualize segmentation results
    autosense::common::publishObjectsMarkers(
        segments_pub_, header, autosense::common::GREEN.rgbA, obsv_objects);

    autosense::tracking::TrackingOptions tracking_options;
    tracking_options.velo2world_trans = velo2world;
    std::vector<autosense::ObjectPtr> tracking_objects_velo;
    autosense::common::Clock clock_tracking;
    tracking_worker_->track(obsv_objects, kTimeStamp, tracking_options,
                            &tracking_objects_velo);

    tracking_worker_->updateDynProp(&tracking_objects_velo,
                                    pub_course_speed_limit);

    if (verbose)
        ROS_INFO_STREAM("Finish tracking. " << tracking_objects_velo.size()
                                            << " Objects Tracked. Took "
                                            << clock_tracking.takeRealTime()
                                            << "ms.");

    std::cout << "Before classify================================" << std::endl;
    classifier_worker_->classify_vector(tracking_objects_velo);
    std::cout << "after classify=================================="
              << std::endl;

    /**
     * publish tracking object clouds for classification
     *   object state: ground center & yaw & velocity
     *   object size, observed segment & its id
     */

    // can_bus publishers
    if (care_object_only) {
        std::vector<autosense::ObjectPtr> objects_care;
        for (const auto &object : tracking_objects_velo)
            if (object->type != autosense::NOTSURE)
                objects_care.push_back(object);
        tracking_objects_velo = objects_care;
    }
    std::vector<autosense::ObjectPtr> objects_id_pub_ =
        id_pub_publisher_->onNewObjects(tracking_objects_velo);
    autosense::common::publishBBoxes(bbox_pub_, header, odom, objects_id_pub_);
    autosense::common::publishLidarCameraObjects(lidar_camera_pub_, header,
                                                 theta, pub_course_speed_limit,
                                                 offset, objects_id_pub_);

    if (visualize) {
        autosense::common::publishObjectsTrackerID(
            tracking_objects_tracker_id_pub_, header,
            autosense::common::RED.rgbA, objects_id_pub_);

        for (const auto &object : objects_id_pub_) {
            std::cout << "ID: " << object->tracker_id
                      << " type: " << object->type
                      << " [l, w, h]: " << object->length << '\t'
                      << object->width << "\t" << object->height << std::endl;
        }

        const std::vector<autosense::ObjectPtr> &tracking_objects_world =
            tracking_worker_->collectTrackingObjectsInWorld();
        autosense::common::publishTrackingObjects(
            tracking_output_objects_pub_, header, tracking_objects_world);
        // publish fixed trajectory for classification
        const std::vector<autosense::FixedTrajectory> &fixed_trajectories =
            tracking_worker_->collectFixedTrajectories();
        autosense::common::publishTrackingFixedTrajectories(
            tracking_output_trajectories_pub_, header, fixed_trajectories);

        // visualize tracking process results, Object Trajectories
        const std::map<autosense::IdType, autosense::Trajectory> &trajectories =
            tracking_worker_->collectTrajectories();
        autosense::common::publishObjectsTrajectory(
            tracking_objects_trajectory_pub_, header, pose.inverse(),
            trajectories);
        autosense::common::publishObjectsMarkers(
            tracking_objects_pub_, header, autosense::common::DARKBLUE.rgbA,
            objects_id_pub_);
        // construct tracking-help segmentation results
        std::vector<autosense::PointICloudPtr> objects_cloud;
        for (size_t idx = 0u; idx < tracking_objects_velo.size(); ++idx) {
            objects_cloud.push_back(tracking_objects_velo[idx]->cloud);
        }
        autosense::common::publishClustersCloud<autosense::PointI>(
            tracking_objects_cloud_pub_, header, objects_cloud);
        // Velocity value and direction
        autosense::common::publishObjectsVelocityArrow(
            tracking_objects_velocity_pub_, header, autosense::common::RED.rgbA,
            tracking_objects_velo);
    }
}

void OnGPS(const boost::shared_ptr<const geometry_msgs::Pose2D> &gps_msg) {
    theta = gps_msg->theta;
    if (verbose) ROS_INFO_STREAM("gps theta: " << theta);
}

// bool srv_pos3d_func(perception_msgs::pos3d::Request &req,
//     perception_msgs::pos3d::Response &res)
// {

//     autosense::PointICloudPtr cloud_combined(new autosense::PointICloud);
//     for(const auto& cloud: *non_ground_copy) *cloud_combined += *cloud;
//     auto m = cloud_combined->getMatrixXfMap(3,4,0);

//     res.lat = 0.0;
//     res.lon = 1.0;
//     res.height = 2.0;
//     return true;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_node");

    // Node handle
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner(1);

    // Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/local_frame_id", local_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/global_frame_id",
                        global_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/tf_timeout_ms", tf_timeout_ms_);

    std::string sub_pcs_segmented_topic;
    int sub_pcs_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_segmented_topic",
                        sub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_queue_size",
                        sub_pcs_queue_size);

    std::string sub_gps_topic;
    private_nh.getParam(param_ns_prefix_ + "/sub_gps_topic", sub_gps_topic);

    std::string pub_segments_coarse_topic, pub_segments_predict_topic,
        pub_segments_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_coarse_topic",
                        pub_segments_coarse_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_predict_topic",
                        pub_segments_predict_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_topic",
                        pub_segments_topic);

    std::string pub_output_objects_topic, pub_output_trajectories_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_output_objects_topic",
                        pub_output_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_output_trajectories_topic",
                        pub_output_trajectories_topic);

    std::string pub_tracking_objects_topic, pub_tracking_objects_cloud_topic,
        pub_tracking_objects_velocity_topic,
        pub_tracking_objects_tracker_id_topic,
        pub_tracking_objects_trajectory_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_topic",
                        pub_tracking_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_cloud_topic",
                        pub_tracking_objects_cloud_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_velocity_topic",
        pub_tracking_objects_velocity_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_tracker_id_topic",
        pub_tracking_objects_tracker_id_topic);
    private_nh.getParam(
        param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic",
        pub_tracking_objects_trajectory_topic);
    private_nh.param<double>(param_ns_prefix_ + "/threshold_contian_IoU",
                             threshold_contian_IoU_, 1.0);

    // Publish settings for the Autodrive challenge
    std::string pub_lidar_camera_topic_;  //, srv_lidar_camera_name;
    private_nh.getParam(param_ns_prefix_ + "/pub_lidar_camera_topic_",
                        pub_lidar_camera_topic_);
    private_nh.getParam(param_ns_prefix_ + "/pub_course_speed_limit",
                        pub_course_speed_limit);
    private_nh.getParam(param_ns_prefix_ + "/offset", offset);
    private_nh.getParam(param_ns_prefix_ + "/care_object_only",
                        care_object_only);
    // private_nh.getParam(param_ns_prefix_ + "/srv_lidar_camera_name",
    // srv_lidar_camera_name);

    int pub_lidar_camera_id_start_;
    int pub_lidar_camera_id_num_;
    private_nh.param<int>(param_ns_prefix_ + "/pub_lidar_camera_id_start_",
                          pub_lidar_camera_id_start_, 1);
    private_nh.param<int>(param_ns_prefix_ + "/pub_lidar_camera_id_num_",
                          pub_lidar_camera_id_num_, 32);

    // Control the command line output
    private_nh.getParam(param_ns_prefix_ + "/verbose", verbose);
    private_nh.getParam(param_ns_prefix_ + "/visualize", visualize);

    // // For projection demonstration and service
    std::vector<double> K_C_vec(9, 0.);
    private_nh.getParam("callibration/K_C", K_C_vec);
    K_C = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(
              K_C_vec.data(), 3, 3, Eigen::OuterStride<>(3))
              .transpose();

    std::vector<double> R_Lidar_CameraC_vec(9, 0.);
    private_nh.getParam("callibration/R_Lidar_CameraC", R_Lidar_CameraC_vec);
    R_Lidar_CameraC =
        Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(
            R_Lidar_CameraC_vec.data(), 3, 3, Eigen::OuterStride<>(3))
            .transpose();

    std::vector<double> t_Lidar_CameraC_vec(3, 0.);
    private_nh.getParam("callibration/t_Lidar_CameraC", t_Lidar_CameraC_vec);
    t_Lidar_CameraC = Eigen::Map<Eigen::Vector3d, 0, Eigen::OuterStride<>>(
        t_Lidar_CameraC_vec.data(), 3, 1, Eigen::OuterStride<>(3));
    // std::cout << "t_Lidar_CameraC\n" << t_Lidar_CameraC << std::endl;

    std::vector<double> D_C(4, 0.);
    private_nh.getParam("callibration/D_C", D_C);
    // D_C = Eigen::Map<Eigen::Vector3d, 0, Eigen::OuterStride<>
    // >(D_C_vec.data(),4,1,Eigen::OuterStride<>(4));

    tracking_params_ = autosense::common::getTrackingWorkerParams(
        private_nh, param_ns_prefix_);
    // ROS_INFO_STREAM("tracking_collect_consecutive_invisible_maximum: " <<
    // tracking_params_.tracking_collect_consecutive_invisible_maximum);

    classifier_params_ =
        autosense::common::getClassfierParams(private_nh, "classifier");
    // ROS_INFO_STREAM("Classifier K_C: " << classifier_params_.visual_K_C);
    // std::cout << "[ped, car, deer]: " <<
    // classifier_params_.volumetric_params.use_car_model << ", " <<
    // classifier_params_.volumetric_params.use_human_model << ", " <<
    // classifier_params_.volumetric_params.use_deer_model << std::endl;

    // Init core compoments
    object_builder_ = autosense::object_builder::createObjectBuilder();
    if (nullptr == object_builder_) {
        ROS_FATAL("Failed to create object_builder_.");
        return -1;
    }
    tracking_worker_ =
        autosense::tracking::createTrackingWorker(tracking_params_);
    if (nullptr == tracking_worker_) {
        ROS_FATAL("Failed to create tracking_worker_.");
        return -1;
    }
    tracking_worker_->verbose = verbose;
    tracking_worker_->setVerbose(verbose);

    classifier_worker_ =
        autosense::classifier::createClassifier(classifier_params_);
    if (nullptr == classifier_worker_) {
        ROS_FATAL("Failed to create classifier.");
        return -1;
    }
    classifier_worker_->verbose = verbose;

    // Init service
    // srv_pos3d = nh.advertiseService(srv_lidar_camera_name, srv_pos3d_func);

    // Init subscribers and publishers
    pcs_segmented_sub_ = nh.subscribe<autosense_msgs::PointCloud2Array>(
        sub_pcs_segmented_topic, sub_pcs_queue_size, OnSegmentClouds);
    tf_listener_.reset(new tf::TransformListener);

    gps_sub_ = nh.subscribe<geometry_msgs::Pose2D>(sub_gps_topic, 1, OnGPS);

    // image_sub_ = nh.subscribe<sensor_msgs::Image>("/camera/center/image_raw",
    // 1, OnImage);

    // segments
    segments_coarse_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_coarse_topic, 1);
    segments_predict_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_segments_predict_topic, 1);
    segments_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
        pub_segments_topic, 1);

    // tracking infos for debugging
    tracking_objects_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_topic, 1);
    tracking_objects_cloud_pub_ =
        private_nh.advertise<sensor_msgs::PointCloud2>(
            pub_tracking_objects_cloud_topic, 1);
    tracking_objects_velocity_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_velocity_topic, 1);
    tracking_objects_tracker_id_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_tracker_id_topic, 1);
    tracking_objects_trajectory_pub_ =
        private_nh.advertise<visualization_msgs::MarkerArray>(
            pub_tracking_objects_trajectory_topic, 1);

    // the whole tracking output
    tracking_output_objects_pub_ =
        private_nh.advertise<autosense_msgs::TrackingObjectArray>(
            pub_output_objects_topic, 1);
    tracking_output_trajectories_pub_ =
        private_nh.advertise<autosense_msgs::TrackingFixedTrajectoryArray>(
            pub_output_trajectories_topic, 1);

    lidar_camera_pub_ = private_nh.advertise<perception_msgs::Objects>(
        pub_lidar_camera_topic_, 1);

    id_pub_publisher_ =
        autosense::common::IdPubManager<autosense::IdPubType>::instantiate(
            pub_lidar_camera_id_start_, pub_lidar_camera_id_num_);

    // For projection demonstration only
    pcs_distort_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>("/cepton/distort", 1);

    std::string sub_nav_topic, pub_bbox_topic;
    int sub_nav_queue_size;

    private_nh.getParam(param_ns_prefix_ + "/sub_nav_topic", sub_nav_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_nav_queue_size",
                        sub_nav_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_bbox_topic", pub_bbox_topic);

    bbox_pub_ =
        nh.advertise<vision_msgs::BoundingBox3DArray>(pub_bbox_topic, 1);

    nav_sub_ = nh.subscribe<nav_msgs::Odometry>(sub_nav_topic,
                                                sub_nav_queue_size, OnNavOdom);

    spiner.start();
    ROS_INFO("tracking_node started...");

    ros::waitForShutdown();
    ROS_INFO("tracking_node exited...");

    return 0;
}
