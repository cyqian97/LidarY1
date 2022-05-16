#include "ism_classifier.hpp"


namespace autosense {
namespace classifier {

ISMClassifier::ISMClassifier() {}

ISMClassifier::ISMClassifier(const ClassifierParams& params): params_(params)
{
    normal_estimator_.setRadiusSearch (params_.ism_normal_estimator_radius);

    fpfh_ = new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >;
    fpfh_->setRadiusSearch (params_.ism_fpfh_radius);

    ism_.setFeatureEstimator(fpfh_);
    ism_.setSamplingSize (params_.ism_sampling_size);  
    ism_.setNumberOfClusters(params_.ism_num_clusters);

    model_  = new pcl::features::ISMModel;
    model_->loadModelFromfile (params_.ism_classifier_model_path);
}

ISMClassifier::~ISMClassifier() {}

void ISMClassifier::classify(const ObjectPtr &object)
{
    ObjectType type_now = NOTSURE;

    if(!object->size_conjectures.empty())
    {   
        pcl::PointCloud<pcl::Normal>::Ptr normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
        normal_estimator_.setInputCloud (object->cloud);
        normal_estimator_.compute (*normals);

        normal_check = true;
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
                pcl::features::ISMVoteList<pcl::PointXYZ>::Ptr vote_list = ism.findObjects (
                    model_,
                    object->cloud,
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

            int maxElementIndex = std::max_element(class_peaks.begin(),class_peaks.end()) - class_peaks.begin();
            int firstIndex = std::distance(class_peaks.begin(), maxElementIndex)
            
            if(class_peaks[firstIndex] > std::numeric_limits<double>::epsilon())
            {
                type_now = size_conjectures[firstIndex];
                
            }
            else
            {
                type_now = NOTSURE;
            }
        }
    }

    std::map<normalsIdType, std::vector<normalsObjectType>>::iterator it_tracker_history 
        = type_histories.find(object->tracker_id);
    if (it_tracker_history != type_histories.end()){
        it_tracker_history->second.push_back(type_now);
    } 
    else {
        std::vector<normalsObjectType> _temp_history{type_now};
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
    std::map<autosense::IdType, std::vector<autosense::ObjectType>>::iterator it_tracker_fixed;
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

    std::map<autosense::IdType, std::vector<autosense::ObjectType>>::iterator it_tracker_history;
    for(const auto& object: objects_label_not_fixed)
    {
        classify(object);

        it_tracker_history = type_histories.find(object->tracker_id);

        if (it_tracker_history->second.size()>30)
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