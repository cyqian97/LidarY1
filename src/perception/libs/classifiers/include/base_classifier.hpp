/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */
#ifndef CLASSIFIERS_INCLUDE_CLASSIFIER_BASE_CLASSIFIER_HPP_
#define CLASSIFIERS_INCLUDE_CLASSIFIER_BASE_CLASSIFIER_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "common/types/type.h"

namespace autosense {
namespace classifier {

class BaseClassifier {
 public:
    /// @brief classify the object    
    virtual void classify_vector(const std::vector<ObjectPtr> &objects_obsved) = 0;

    virtual void classify(const ObjectPtr &object)  = 0;
 
 private:
    virtual std::map<IdType, std::vector<ObjectType>> type_histories const = 0;
    virtual std::map<IdType, ObjectType> type_fixed const = 0;

};  // BaseClassifier

}  // namespace classifier
}  // namespace autosense

#endif  // CLASSIFIERS_INCLUDE_CLASSIFIER_BASE_CLASSIFIER_HPP_
