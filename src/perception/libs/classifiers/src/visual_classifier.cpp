#include "classifiers/visual_classifier.hpp"

namespace autosense
{
    namespace classifier
    {

        VisualClassifier::VisualClassifier() {}

        VisualClassifier::VisualClassifier(const ClassifierParams &params) : params_(params)
        {
            sub_bboxes_ = nh_.subscribe("/target_detection", 1,
                                        &VisualClassifier::updateBBoxes, this);
            ROS_INFO_STREAM("params_.visual_x1: " << params_.visual_x1);
            ROS_INFO_STREAM("params_.visual_x2: " << params_.visual_x2 << std::endl);
            ROS_INFO_STREAM("params_.visual_y1: " << params_.visual_y1 << std::endl);
            ROS_INFO_STREAM("params_.visual_y2: " << params_.visual_y2 << std::endl);
        }

        VisualClassifier::~VisualClassifier() {}

        void VisualClassifier::updateBBoxes(
            const boost::shared_ptr<const perception_msgs::yolo_boxes> bboxes_msg)
        {
            std::vector<perception_msgs::yolo_box> _bboxes;

            std::map<ClassificationType, ObjectType>::iterator _it_coco_class_map_;

            for (const perception_msgs::yolo_box &bbox : bboxes_msg->bounding_boxes)
            {
                ClassificationType t_ = bbox.obj_class[0];
                _it_coco_class_map_ = coco_class_map_.find(t_);
                if (_it_coco_class_map_ != coco_class_map_.end())
                    _bboxes.push_back(bbox);
            }

            bboxes = boost::make_shared<std::vector<perception_msgs::yolo_box>>(_bboxes);
        }

        void VisualClassifier::classify(const ObjectPtr &object)
        {
            ObjectType type_now = NOTSURE;

            if (!object->size_conjectures.empty())
            {
                // ROS_INFO_STREAM("Enter classify");
                // Check is the size fits a traffic blockage
                // Traffic blockage does not go into further classification
                bool _find_traffic_blockage = false;
                if (!_find_traffic_blockage)
                {
                    auto iter_conjectures = std::find(
                        object->size_conjectures.begin(), object->size_conjectures.end(), autosense::CONE);
                    if (iter_conjectures != object->size_conjectures.end())
                    {
                        type_now = autosense::CONE;
                        _find_traffic_blockage = true;
                    }
                }
                if (!_find_traffic_blockage)
                {
                    auto iter_conjectures = std::find(
                        object->size_conjectures.begin(), object->size_conjectures.end(), autosense::BARRICADE);
                    if (iter_conjectures != object->size_conjectures.end())
                    {
                        type_now = autosense::BARRICADE;
                        _find_traffic_blockage = true;
                    }
                }
                if (!_find_traffic_blockage)
                {
                    auto iter_conjectures = std::find(
                        object->size_conjectures.begin(), object->size_conjectures.end(), autosense::DEER);
                    if (iter_conjectures != object->size_conjectures.end())
                    {
                        type_now = autosense::DEER;
                        _find_traffic_blockage = true;
                    }
                }

                // type_now = object->size_conjectures[0];

                // Visaul classification
                if (!_find_traffic_blockage)
                {
                    if (verbose)
                    {
                        ROS_INFO_STREAM("Tracker ID: " << object->tracker_id << ", size conj:");
                        for (const auto &i_class : object->size_conjectures)
                            ROS_INFO_STREAM("\t" << int(i_class));
                    }

                    if (bboxes != nullptr)
                    {
                        // ROS_INFO_STREAM("Start proj");

                        auto _x = object->cloud->getMatrixXfMap(3, 4, 0);
                        Eigen::MatrixXd x = _x.cast<double>();
                        Eigen::MatrixXd res = autosense::common::calibration::proj(
                            params_.visual_K_C, params_.visual_R_Lidar_CameraC,
                            params_.visual_t_Lidar_CameraC, params_.visual_D_C,
                            x);

                        std::map<ClassificationType, int> _classes_counts;
                        std::map<ClassificationType, int>::iterator _it_classes_counts;
                        int _current_max_count = 0;
                        int _in_window_count = 0;
                        ClassificationType _current_max_class;
                        for (int i = 0; i < res.cols(); i++)
                        {
                            // Check if point is inside the cropped image
                            if (res(0, i) < params_.visual_x1 || res(0, i) > params_.visual_x2 ||
                                res(1, i) < params_.visual_y1 || res(1, i) > params_.visual_y2)
                                continue;
                            _in_window_count++;

                            for (const auto &bbox : *bboxes)
                            {
                                if (res(0, i) > bbox.xmin + 1000 && res(0, i) < bbox.xmax + 1000 &&
                                    res(1, i) > bbox.ymin + 1000 && res(1, i) < bbox.ymax + 1000)
                                {

                                    ClassificationType t_ = bbox.obj_class[0];
                                    _it_classes_counts = _classes_counts.find(t_);
                                    if (_it_classes_counts == _classes_counts.end())
                                    {
                                        _classes_counts.insert(std::make_pair(t_, 1));
                                    }
                                    else
                                    {
                                        _it_classes_counts->second++;
                                    }
                                    if (_it_classes_counts->second > _current_max_count)
                                    {
                                        _current_max_count = _it_classes_counts->second;
                                        _current_max_class = _it_classes_counts->first;
                                    }
                                }
                            }
                        }
                        ROS_INFO_STREAM("Tracker ID: " << object->tracker_id << ", size conj:");
                        ROS_INFO_STREAM("current_max_count: " << _current_max_count);
                        ROS_INFO_STREAM("current_total_num: " << _in_window_count);
                        if (_current_max_count > params_.visual_thld_ratio * _in_window_count)
                        {
                            std::map<ClassificationType, ObjectType>::iterator _it_coco_class_map_ = coco_class_map_.find(_current_max_class);
                            if (_it_coco_class_map_ != coco_class_map_.end())
                            {
                                auto iter_conjectures = std::find(
                                    object->size_conjectures.begin(), object->size_conjectures.end(),
                                    _it_coco_class_map_->second);
                                if (iter_conjectures != object->size_conjectures.end())
                                    type_now = _it_coco_class_map_->second;
                            }

                            ROS_INFO_STREAM("type now: " << int(type_now));
                        }
                    }
                }
            }

            std::map<IdType, std::vector<ObjectType>>::iterator it_tracker_history = type_histories.find(object->tracker_id);
            if (it_tracker_history != type_histories.end())
            {
                it_tracker_history->second.push_back(type_now);
            }
            else
            {
                std::vector<ObjectType> _temp_history{type_now};
                type_histories.insert(std::make_pair(object->tracker_id, _temp_history));
            }
        }

        void VisualClassifier::sizeConjectures(const std::vector<ObjectPtr> &objects_obsved)
        {
            if (params_.volumetric_params.use_cone_model)
                roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_cone);

            if (params_.volumetric_params.use_barricade_model)
                roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_barricade);

            if (params_.volumetric_params.use_car_model)
                roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_car);

            if (params_.volumetric_params.use_human_model)
                roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_human);

            if (params_.volumetric_params.use_deer_model)
                roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_deer);
        }

        void VisualClassifier::classify_vector(const std::vector<ObjectPtr> &objects_obsved)
        {
            // Assign type to objects whose types are already fixed
            std::vector<ObjectPtr> objects_label_not_fixed;
            std::map<autosense::IdType, autosense::ObjectType>::iterator it_tracker_fixed;
            for (const auto &object : objects_obsved)
            {
                it_tracker_fixed = type_fixed.find(object->tracker_id);
                if (it_tracker_fixed != type_fixed.end())
                {
                    object->type = it_tracker_fixed->second;
                }
                else
                {
                    objects_label_not_fixed.push_back(object);
                }
            }
            
            // Project each cloud to camera frame
            std::vector<Eigen::MatrixXd> objects_projected;
            for (const auto &object : objects_label_not_fixed)
            {
                auto _x = object->cloud->getMatrixXfMap(3, 4, 0);
                Eigen::MatrixXd x = _x.cast<double>();
                Eigen::MatrixXd res = autosense::common::calibration::proj(
                    params_.visual_K_C, params_.visual_R_Lidar_CameraC,
                    params_.visual_t_Lidar_CameraC, params_.visual_D_C, x);
                objects_projected.push_back(res);
            }


            if (bboxes != nullptr && bboxes->size() > 0)
            {
                for (const auto &bbox : *bboxes)
                {
                    std::cout << "BBOX:" << bbox.obj_class[0] << std::endl;
                    // Map the class to what we need
                    std::map<ClassificationType, ObjectType>::iterator _it_coco_class_map_ =
                        coco_class_map_.find(bbox.obj_class[0]);
                    ObjectType type_now = _it_coco_class_map_->second;
                    if (_it_coco_class_map_->second == PEDESTRIAN ||
                        _it_coco_class_map_->second == CAR ||
                        _it_coco_class_map_->second == DEER)
                    {
                        std::vector<int> obj_in_ids;
                        std::vector<double> obj_in_sizes;

                        for (int i_obj = 0; i_obj < objects_label_not_fixed.size(); i_obj++)
                        {

                            int _in_box_count = 0;
                            int _in_window_count = 0;

                            Eigen::MatrixXd res = objects_projected[i_obj];
                            double _obj_x_min = res(0, 0);
                            double _obj_x_max = res(0, 0);
                            double _obj_y_min = res(1, 0);
                            double _obj_y_max = res(1, 0);

                            for (int i = 0; i < res.cols(); i++)
                            {
                                if (res(0, i) < _obj_x_min)
                                    _obj_x_min = res(0, i);
                                if (res(0, i) > _obj_x_max)
                                    _obj_x_max = res(0, i);
                                if (res(1, i) < _obj_y_min)
                                    _obj_y_min = res(1, i);
                                if (res(1, i) > _obj_y_max)
                                    _obj_y_max = res(1, i);
                                // Check if point is inside the cropped image
                                if (res(0, i) < params_.visual_x1 || res(0, i) > params_.visual_x2 ||
                                    res(1, i) < params_.visual_y1 || res(1, i) > params_.visual_y2)
                                    continue;

                                _in_window_count++;

                                if (res(0, i) > bbox.xmin + params_.visual_x1 && res(0, i) < bbox.xmax + params_.visual_x1 &&
                                    res(1, i) > bbox.ymin + params_.visual_y1 && res(1, i) < bbox.ymax + params_.visual_y1)
                                    _in_box_count++;
                            }
                            if (_in_box_count > params_.visual_thld_ratio * _in_window_count)
                            {
                                std::cout << "\tid " << i_obj << std::endl;
                                std::cout << "\tratio " << (double)_in_box_count / (double)_in_window_count << std::endl;
                                std::cout << "\tsize " << (_obj_x_max - _obj_x_min) * (_obj_y_max - _obj_y_min) << std::endl;

                                obj_in_ids.push_back(i_obj);
                                obj_in_sizes.push_back((_obj_x_max - _obj_x_min) * (_obj_y_max - _obj_y_min));
                            }
                        }

                        double max_size = 0;
                        int obj_id_select = -1;
                        for (int i = 0; i < obj_in_ids.size(); i++)
                        {
                            if (obj_in_sizes[i] > max_size)
                            {
                                max_size = obj_in_sizes[i];
                                obj_id_select = obj_in_ids[i];
                            }
                        }

                        std::cout << "\tselect id " << obj_id_select << std::endl;
                        if (obj_id_select > 0)
                        {
                            objects_label_not_fixed[obj_id_select]->type = type_now;
                            std::map<IdType, std::vector<ObjectType>>::iterator it_tracker_history =
                                type_histories.find(objects_label_not_fixed[obj_id_select]->tracker_id);
                            if (it_tracker_history != type_histories.end())
                            {
                                it_tracker_history->second.push_back(type_now);
                            }
                            else
                            {
                                std::vector<ObjectType> _temp_history{type_now};
                                type_histories.insert(std::make_pair(objects_label_not_fixed[obj_id_select]->tracker_id, _temp_history));
                            }
                            objects_label_not_fixed.erase(objects_label_not_fixed.begin() + obj_id_select);
                            objects_projected.erase(objects_projected.begin() + obj_id_select);
                        }
                    }
                }
            }
            // Size based classification
            // Get possible types from size
            sizeConjectures(objects_label_not_fixed);

            for (const auto &object : objects_label_not_fixed)
            {

                bool _find_traffic_blockage = false;
                ObjectType type_now = NOTSURE;

                if (!_find_traffic_blockage)
                {
                    auto iter_conjectures = std::find(
                        object->size_conjectures.begin(), object->size_conjectures.end(), autosense::CONE);
                    if (iter_conjectures != object->size_conjectures.end())
                    {
                        type_now = autosense::CONE;
                        _find_traffic_blockage = true;
                        object->type = autosense::CONE;
                    }
                }
                if (!_find_traffic_blockage)
                {
                    auto iter_conjectures = std::find(
                        object->size_conjectures.begin(), object->size_conjectures.end(), autosense::BARRICADE);
                    if (iter_conjectures != object->size_conjectures.end())
                    {
                        type_now = autosense::BARRICADE;
                        _find_traffic_blockage = true;
                        object->type = autosense::BARRICADE;
                    }
                }
                std::map<IdType, std::vector<ObjectType>>::iterator it_tracker_history =
                    type_histories.find(object->tracker_id);
                if (it_tracker_history != type_histories.end())
                {
                    it_tracker_history->second.push_back(type_now);
                }
                else
                {
                    std::vector<ObjectType> _temp_history{type_now};
                    type_histories.insert(std::make_pair(object->tracker_id, _temp_history));
                }
            }
            // Manager historyt based class fixing
            std::map<autosense::IdType, std::vector<autosense::ObjectType>>::iterator it_tracker_history;
            for (const auto &object : objects_label_not_fixed)
            {
                // classify(object);

                it_tracker_history = type_histories.find(object->tracker_id);

                if (it_tracker_history->second.size() > params_.abort_frame_lim)
                {
                    type_fixed.insert(std::make_pair(object->tracker_id, NOTSURE));
                }
                else if (it_tracker_history->second.size() >= params_.fix_frame_lim)
                {
                    std::vector<autosense::ObjectType> type_history_last = std::vector<autosense::ObjectType>(it_tracker_history->second.end() - params_.fix_frame_lim, it_tracker_history->second.end());

                    bool is_label_stable = true;
                    for (const auto &_temp_type : type_history_last)
                    {
                        // Fix label that comes from visual detection, since visual detection may miss objects
                        // For Lidar only detections, since we are not very confident, we donot fix them
                        if (_temp_type == NOTSURE ||
                            _temp_type != type_history_last[0])
                        {
                            is_label_stable = false;
                            break;
                        }
                    }
                    if (is_label_stable)
                    {
                        object->type = type_history_last[0];
                        type_fixed.insert(std::make_pair(object->tracker_id, type_history_last[0]));
                    }
                }
            }
        }

    } // classifier
} // autosense
