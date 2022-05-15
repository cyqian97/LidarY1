/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef CLASSIFIERS_INCLUDE_CLASSIFIERS_EUCLIDEAN_CLASSIFIER_HPP_
#define CLASSIFIERS_INCLUDE_CLASSIFIERS_EUCLIDEAN_CLASSIFIER_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

#include "common/types/type.h"
#include "roi_filters/roi.hpp"     

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/recognition/implicit_shape_model.h>
#include <pcl/recognition/impl/implicit_shape_model.hpp>



namespace autosense {
namespace classifier {

class ISMClassifier : public BaseClassifier {
 public:
    ISMClassifier();

    explicit ISMClassifier(const ClassifierParams& params);

    ~ISMClassifier();

    /// @brief classify the object using the implicit shape model.
    virtual bool classify(const ObjectPtr &object_obsved) = 0;

    virtual std::string name() const { return "ISMClassifier"; }

 private:
    ClassifierParams params_;

    std::map<autosense::IdType, std::vector<autosense::ObjectType>> type_histories;
    std::map<autosense::IdType, autosense::ObjectType> type_fixed;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >::Ptr fpfh_;
    pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal> ism_;  
    pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal>::ISMModelPtr model_;
};  // class EuclideanSegmenter

}  // namespace ISMClassifier
}  // namespace autosense

#endif  // CLASSIFIERS_INCLUDE_CLASSIFIERS_EUCLIDEAN_CLASSIFIER_HPP_
