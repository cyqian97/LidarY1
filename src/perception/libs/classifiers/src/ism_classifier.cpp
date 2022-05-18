#include "classifiers/ism_classifier.hpp"


namespace autosense {
namespace classifier {

ISMClassifier::ISMClassifier() {}

ISMClassifier::ISMClassifier(const ClassifierParams& params): params_(params)
{
    normal_estimator_.setRadiusSearch (params_.ism_normal_estimator_radius);

    
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >::Ptr _temp_fpfh
        (new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >);
    fpfh_ = _temp_fpfh;
    ROS_INFO_STREAM("Classifier type: " << params_.classifier_type);
    ROS_INFO_STREAM("ism_vote_sigma_multiplier: " << params_.ism_vote_sigma_multiplier);
    ROS_INFO_STREAM("ism_fpfh_radius: " << params_.ism_fpfh_radius);
    fpfh_->setRadiusSearch (params_.ism_fpfh_radius);

    ism_.setFeatureEstimator(fpfh_);
    ism_.setSamplingSize (params_.ism_sampling_size);  
    ism_.setNumberOfClusters(params_.ism_num_clusters);

    // model_  = new pcl::features::ISMModel;

    pcl::ism::ImplicitShapeModelEstimation<125, pcl::PointXYZ, pcl::Normal>::ISMModelPtr _temp_model (new pcl::features::ISMModel);
    model_=_temp_model;
    ROS_INFO_STREAM("Model path: " << params_.classifier_model_path);
    model_->loadModelFromfile (params_.classifier_model_path);
    ROS_INFO_STREAM( "Model class number: " << model_->classes_.size());

    // std::cout << "[ped, car, deer]: " << params_.volumetric_params.use_car_model << ", " << params_.volumetric_params.use_human_model << ", " << params_.volumetric_params.use_deer_model << std::endl;
}

ISMClassifier::~ISMClassifier() {}

void ISMClassifier::classify(const ObjectPtr &object)
{
    ObjectType type_now = NOTSURE;

    if(!object->size_conjectures.empty())
    {   
        pcl::PointCloud<pcl::Normal>::Ptr normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr _temp_cloud(new pcl::PointCloud<pcl::PointXYZ> ());


        common::convertPointCloud(object->cloud,_temp_cloud);
        normal_estimator_.setInputCloud (_temp_cloud);
        normal_estimator_.compute (*normals);

        bool normal_check = true;
        for (int i = 0; i < normals->size(); i++)
        {
            if (!pcl::isFinite<pcl::Normal>((*normals)[i]))
            {
                normal_check = false;
                break;
            }
        }
        if (normal_check)
        {
            std::vector<double> class_peaks; 

            for(const auto& i_class: object->size_conjectures) 
            {
                common::convertPointCloud(object->cloud,_temp_cloud);
                pcl::features::ISMVoteList<pcl::PointXYZ>::Ptr vote_list;
                
                vote_list = ism_.findObjects (
                model_,
                _temp_cloud,
                normals,
                int(i_class));


                // std::cout << "Class: " << i_class << ", vote size: " << vote_list->getNumberOfVotes() << std::endl;
                if (vote_list->getNumberOfVotes()<1) 
                {
                    class_peaks.push_back(0.0);
                }
                else
                {
                    double radius = model_->sigmas_[int(i_class)] * params_.ism_vote_radius_multiplier;
                    double sigma = model_->sigmas_[int(i_class)] * params_.ism_vote_sigma_multiplier;

                    std::vector<pcl::ISMPeak, Eigen::aligned_allocator<pcl::ISMPeak> > strongest_peaks;
                    vote_list->findStrongestPeaks (strongest_peaks, int(i_class), radius, sigma);
                    class_peaks.push_back(strongest_peaks[0].density);
                }
            }
            std::cout << "=======peaks: ";
            for(const auto& peak: class_peaks)
            {
                std::cout << std::setw(10) << peak;
            }
            std::cout << "\n";
            
            std::vector<double>::iterator maxElementIndex = std::max_element(class_peaks.begin(),class_peaks.end());
            int firstIndex = std::distance(class_peaks.begin(), maxElementIndex);
            
            if(class_peaks[firstIndex] > std::numeric_limits<double>::epsilon())
            {
                type_now = object->size_conjectures[firstIndex];
            }
            else
            {
                type_now = NOTSURE;
            }
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

void ISMClassifier::sizeConjectures(const std::vector<ObjectPtr> &objects_obsved) 
{
    if (params_.volumetric_params.use_car_model)
        roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_car);
    if (params_.volumetric_params.use_human_model)
        roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_human);
    if (params_.volumetric_params.use_deer_model)
        roi::VolumetricFilter(objects_obsved, params_.volumetric_params.model_deer);
}

void ISMClassifier::classify_vector(const std::vector<ObjectPtr> &objects_obsved)
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
        std::cout << "Object ID: " << 
        classify(object);

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
