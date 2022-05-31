/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef CLASSIFIER_INCLUDE_CLASSIFIER_CLASSIFIER_MANAGER_HPP_
#define CLASSIFIER_INCLUDE_CLASSIFIER_CLASSIFIER_MANAGER_HPP_

#include <memory>

#include "classifiers/base_classifier.hpp"
#include "classifiers/ism_classifier.hpp"
#include "classifiers/visual_classifier.hpp"

namespace autosense {
namespace classifier {

static std::unique_ptr<BaseClassifier> createClassifier(
    const ClassifierParams &params) {
    std::unique_ptr<BaseClassifier> classifier_worker;
    if (params.classifier_type == "implicit_shape_model")
    {
        classifier_worker =
            std::unique_ptr<BaseClassifier>(new ISMClassifier(params));
        std::cout << "Classifier type: " << "implicit_shape_model" << std::endl;
    } 
    else if (params.classifier_type == "visual_classifier")
    {
        classifier_worker =
            std::unique_ptr<BaseClassifier>(new VisualClassifier(params));
        std::cout << "Classifier type: " << "visual_classifier" << std::endl;
    }
    return classifier_worker;
}

}  // namespace tracking
}  // namespace autosense

#endif  // TRACKING_INCLUDE_TRACKING_TRACKING_WORKER_MANAGER_HPP_
