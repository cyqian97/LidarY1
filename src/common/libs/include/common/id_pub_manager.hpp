/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */

#ifndef COMMON_INCLUDE_ID_PUB_MANAGER_HPP_
#define COMMON_INCLUDE_ID_PUB_MANAGER_HPP_


#include <map>
#include <string>
#include <vector>
#include "common/types/type.h"  


namespace autosense {
namespace common {

template <typename IdPubType_T>
class IdPubManager {
    public:
        static IdPubManager creat(IdPubType_T const m)
        {   

            typename std::vector<IdPubType_T> id_pub_all(m);
            std::iota(id_pub_all.begin(), id_pub_all.end(), 1);
            return IdPubManager(m,id_pub_all);
        }

/**
 * @brief map the objects' trakcer_id to id_pup
 * @note objects with type NOTSURE will have lower priority.
 * @param objects_in
 * @return objects_out
 */
        std::vector<ObjectPtr> onNewObjects(std::vector<ObjectPtr> objects_in)
        {
            std::vector<ObjectPtr> objects_out; //output objects vector
            std::vector<ObjectPtr> _list_prior; // new objects, type != NOTSURE
            std::vector<ObjectPtr> _list_non_prior; // old objects, type == NOTSURE
            std::vector<ObjectPtr> _list_wait; // new objects, type == NOTSURE

            const typename std::map<IdType,IdPubType_T> _copy_map_tracker_pub(map_tracker_pub_);
            map_tracker_pub_.clear();
            id_pub_available_ = id_pub_all_;

            typename std::map<IdType,IdPubType_T>::iterator _iter_tracker_pub;
            for(const auto &object: objects_in)
            {
                _iter_tracker_pub = _copy_map_tracker_pub.find(object->tracker_id);
                if (_iter_tracker_pub != _copy_map_tracker_pub.end())
                {
                    if(object->type == NOTSURE)
                    {
                        updateIDPub(_iter_tracker_pub);
                        _list_non_prior.push_back(object);
                    }
                    else
                    {
                        updateIDPub(_iter_tracker_pub);
                        object->tracker_id = _iter_tracker_pub->second;
                        objects_out.push_back(object);
                    }
                }
                else
                {
                    if(object->type == NOTSURE)
                    {
                        _list_wait.push_back(object);
                    }
                    else
                    {
                        _list_prior.push_back(object);
                    }
                }
            }

            for(const auto &object: _list_prior)
            {
                if(id_pub_available_.size()>0)
                {
                    IdPubType_T _id_pub = id_pub_available_[0];
                    updateIDPub(object->tracker_id, _id_pub);
                    object->tracker_id = _id_pub;
                    objects_out.push_back(object);
                }
                else if(_list_non_prior.size()>0)
                {
                    IdType _tracker_id = (*_list_non_prior.begin())->tracker_id;
                    _iter_tracker_pub = map_tracker_pub_.find(_tracker_id);
                    _iter_tracker_pub->first = object->tracker_id;
                    object->tracker_id = _iter_tracker_pub->second;
                    objects_out.push_back(object);
                    _list_non_prior.erase(_list_non_prior.begin());
                }
                else
                {
                    break;
                }
            }

            for(const auto &object: _list_non_prior)
            {
                _iter_tracker_pub = map_tracker_pub_.find(object->tracker_id);
                object->tracker_id = _iter_tracker_pub->second;
                objects_out.push_back(object);
            }

            for(const auto &object: _list_wait)
            {
                if(id_pub_available_.size()>0)
                {
                    IdPubType_T _id_pub = id_pub_available_[0];
                    updateIDPub(object->tracker_id, _id_pub);
                    object->tracker_id = _id_pub;
                    objects_out.push_back(object);
                }
                else
                {
                    break;
                }
            }
            return objects_out;
        }
    private:
        const IdPubType_T id_pub_max_;
        const typename std::vector<IdPubType_T> id_pub_all_;
        typename std::vector<IdPubType_T> id_pub_available_;
        typename std::map<IdType,IdPubType_T> map_tracker_pub_;


        IdPubManager(IdPubType_T m, typename std::vector<IdPubType_T> all): id_pub_max_(m), id_pub_all_(all){}


        void updateIDPub(const typename std::map<IdType,IdPubType_T>::iterator iter_tracker_pub)
        {
            map_tracker_pub_.insert(*iter_tracker_pub);
            id_pub_available_.erase(
                std::remove(id_pub_available_.begin(), id_pub_available_.end(), iter_tracker_pub->second), 
                id_pub_available_.end());
        }

        void updateIDPub(const IdType tracker_id, const IdPubType_T id_pub)
        {
            map_tracker_pub_.insert(std::make_pair(tracker_id,id_pub));
            id_pub_available_.erase(
                std::remove(id_pub_available_.begin(), id_pub_available_.end(), id_pub), 
                id_pub_available_.end());
        }
}; // class IdPubManager
}  // namespace common
}  // namespace autosense

#endif  // COMMON_INCLUDE_ID_PUB_MANAGER_HPP_
