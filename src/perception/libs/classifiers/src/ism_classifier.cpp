#include "ism_classifier"

namespace autosense {
namespace classifier {

ISMClassifier::ISMClassifier() {}

ISMClassifier::ISMClassifier(const ClassifierParams& params): params_(params)
{
    fpfh_ = new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<125> >;
    fpfh_->setRadiusSearch (params_.fpfh_radius);

    ism.setFeatureEstimator(fpfh_);
    ism.setSamplingSize (params_.ism_sampling_size);  
    ism.setNumberOfClusters(params_.ism_num_clusters);

    model  = new pcl::features::ISMModel;
    model->loadModelFromfile (params_.classifier_model_path);

}

ISMClassifier::~ISMClassifier() {}

void ISMClassifier::classify(const ObjectPtr &object)
{
    ObjectType type_now = NOTSURE;

    if(!object->size_conjectures.empty())
    {   
        pcl::PointCloud<pcl::Normal>::Ptr normals = (new pcl::PointCloud<pcl::Normal>)->makeShared ();
        normal_estimator.setInputCloud (object->cloud);
        normal_estimator.compute (*normals);

        normal_check = true;
        for (int i = 0; i < normals->size(); i++)
        {
            if (!pcl::isFinite<pcl::Normal>((*normals)[i]))
            {
                normal_check = false;
                break;
            }
        }
        if (!normal_check)
        {
            for(const auto& i_class: object->size_conjectures) {
                
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
}
}