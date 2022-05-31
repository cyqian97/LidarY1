#include "classifiers/visual_classifier.hpp"


namespace autosense {
namespace classifier {

VisualClassifier::VisualClassifier() {}

VisualClassifier::VisualClassifier(const ClassifierParams& params): params_(params)
{
    // normal_estimator_.setRadiusSearch (params_.ism_normal_estimator_radius);

    
    // pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >::Ptr _temp_fpfh
    //     (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >);
    // fpfh_ = _temp_fpfh;
    // ROS_INFO_STREAM("Classifier type: " << params_.classifier_type);
    // ROS_INFO_STREAM("ism_vote_sigma_multiplier: " << params_.ism_vote_sigma_multiplier);
    // ROS_INFO_STREAM("ism_fpfh_radius: " << params_.ism_fpfh_radius);
    // fpfh_->setRadiusSearch (params_.ism_fpfh_radius);

    // ism_.setFeatureEstimator(fpfh_);
    // ism_.setSamplingSize (params_.ism_sampling_size);  
    // ism_.setNumberOfClusters(params_.ism_num_clusters);

    // // model_  = new pcl::features::ISMModel;

    // pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal>::ISMModelPtr _temp_model (new pcl::features::ISMModel);
    // model_=_temp_model;
    // ROS_INFO_STREAM("Model path: " << params_.classifier_model_path);
    // model_->loadModelFromfile (params_.classifier_model_path);
    // ROS_INFO_STREAM( "Model class number: " << model_->classes_.size());

    // std::cout << "[ped, car, deer]: " << params_.volumetric_params.use_car_model << ", " << params_.volumetric_params.use_human_model << ", " << params_.volumetric_params.use_deer_model << std::endl;
}

VisualClassifier::~VisualClassifier() {}

void VisualClassifier::classify(const ObjectPtr &object,
                                const darknet_ros_msgs::BoundingBoxes bboxes)
{
    ObjectType type_now = NOTSURE;
    
    if(!object->size_conjectures.empty())
    {   
        bool _find_traffic_blockage = false;
        if (!_find_traffic_blockage) 
        {
            auto iter_conjectures = std::find(
                object->size_conjectures.begin(),object->size_conjectures.end(),autosense::CONE);
            if(iter_conjectures != object->size_conjectures.end())
            {
                type_now = autosense::CONE;
                _find_traffic_blockage = true;
            }
        }
        if (!_find_traffic_blockage) 
        {
            auto iter_conjectures = std::find(
                object->size_conjectures.begin(),object->size_conjectures.end(),autosense::BARRICADE);
            if(iter_conjectures != object->size_conjectures.end())
            {
                type_now = autosense::BARRICADE;
                _find_traffic_blockage = true;
            }
        }
        
        if (!_find_traffic_blockage) 
        {
            if(verbose)
            {
                std::cout << "Tracker ID: " << object->tracker_id << ", size conj:";
                for(const auto& i_class: object->size_conjectures) std::cout << int(i_class) << " ";
                std::cout << "\n";
            }

            auto _x = object->cloud->getMatrixXfMap(3,4,0);
            Eigen::MatrixXd x = _x.cast <double> ();
            // Eigen::MatrixXd res = autosense::common::calibration::proj(
                // K_C, R_Lidar_CameraC, t_Lidar_CameraC, D_C, x);
            

        }
    }

    std::map<IdType, std::vector<ObjectType>>::iterator it_tracker_history = type_histories.find(object->tracker_id);
    if (it_tracker_history != type_histories.end()){
        it_tracker_history->second.push_back(type_now);
    } 
    else {
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

void VisualClassifier::classify_vector(const std::vector<ObjectPtr> &objects_obsved,
                                    const darknet_ros_msgs::BoundingBoxes bboxes)
{
    std::vector<ObjectPtr> objects_label_not_fixed;
    std::map<autosense::IdType, autosense::ObjectType>::iterator it_tracker_fixed;
    for(const auto& object: objects_obsved)
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
    sizeConjectures(objects_label_not_fixed);
    // for(const auto& object: objects_obsved)
    // {
    //     ROS_INFO_STREAM("Object type: " << int(object->size_conjectures.size()));
    // }


    std::map<autosense::IdType, std::vector<autosense::ObjectType>>::iterator it_tracker_history;
    for(const auto& object: objects_label_not_fixed)
    {
        classify(object,bboxes);

        it_tracker_history = type_histories.find(object->tracker_id);

        if (it_tracker_history->second.size()>params_.abort_frame_lim)
        {
            type_fixed.insert(std::make_pair(object->tracker_id, NOTSURE)); 
        } 
        else if (it_tracker_history->second.size() >= params_.fix_frame_lim)
        {
            std::vector<autosense::ObjectType> type_history_last 
                = std::vector<autosense::ObjectType>(it_tracker_history->second.end() - params_.fix_frame_lim, it_tracker_history->second.end());
            
            bool is_label_stable = true;
            for(const auto& _temp_type: type_history_last) 
            {
                if(_temp_type == NOTSURE || _temp_type != type_history_last[0])
                {
                    is_label_stable = false;
                    break;
                }
            }
            if(is_label_stable)
            {
                object->type = type_history_last[0];
                type_fixed.insert(std::make_pair(object->tracker_id,  type_history_last[0])); 
            }
        }
    }
}


} // classifier
} // autosense
