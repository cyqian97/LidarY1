/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef CLASSIFIERS_INCLUDE_CLASSIFIERS_VISUAL_CLASSIFIER_HPP_
#define CLASSIFIERS_INCLUDE_CLASSIFIERS_VISUAL_CLASSIFIER_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>
#include <Eigen/Core>


#include "common/common.hpp"
#include "common/types/type.h"
#include "common/calibration.hpp"
#include "roi_filters/roi.hpp"  
#include "classifiers/base_classifier.hpp"   
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

// #include <pcl/io/pcd_io.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/feature.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/features/impl/fpfh.hpp>
// #include <pcl/recognition/implicit_shape_model.h>
// #include <pcl/recognition/impl/implicit_shape_model.hpp>



namespace autosense {
namespace classifier {

class VisualClassifier : public BaseClassifier {
 public:
    VisualClassifier();

    explicit VisualClassifier(const ClassifierParams& params);

    ~VisualClassifier();

    /// @brief classify the object using the implicit shape model.
    virtual void classify_vector(const std::vector<ObjectPtr> &objects_obsved);

    virtual void classify(const ObjectPtr &object);

    virtual void sizeConjectures(const std::vector<ObjectPtr> &objects_obsved);

    virtual std::string name() const { return "VisualClassifier"; }

    void updateBBoxes(darknet_ros_msgs::BoundingBoxes bboxes_new) 
     { *bboxes = bboxes_new; }

   //  virtual void setVerbose(const bool _verbose){ verbose = _verbose;}

 private:
   //  bool verbose;

    ClassifierParams params_;
    VolumetricModelParams volumetric_params_;

    std::map<IdType, std::vector<ObjectType>> type_histories;
    std::map<IdType, ObjectType> type_fixed;


    darknet_ros_msgs::BoundingBoxes* bboxes = nullptr;

    std::map<std::string, ObjectType> coco_class_map_{{"person", PEDESTRIAN}, {"car", CAR}, {"truck",CAR},
       {"cat", DEER}, {"dog", DEER}, {"horse", DEER}, {"sheep", DEER}, {"cow", DEER}, {"elephant", DEER}, {"bear", DEER}, {"zebra", DEER}, {"giraffe",DEER}};

  //   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;  
  //   pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >::Ptr fpfh_;
  //   pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal> ism_;  
  //  //  pcl::Feature< pcl::PointXYZ, pcl::Histogram<125> >::Ptr feature_estimator_;
  //   pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model_;
};  // class VisualClassifier

}  // namespace VisualClassifier
}  // namespace autosense

#endif  // CLASSIFIERS_INCLUDE_CLASSIFIERS_VISUAL_CLASSIFIER_HPP_
