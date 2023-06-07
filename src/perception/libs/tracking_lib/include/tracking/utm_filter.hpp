
#include <ros/ros.h>
#include "common/types/object.hpp"               
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

// Filter segmented object according to theri utm
void utmFilter(std::vector<autosense::ObjectPtr> *objects_array, tf2::Transform trans_tf2) {
    std::vector<autosense::ObjectPtr> temp_array;
    for (auto& object : *objects_array) {
        tf2::Vector3 v2(object->ground_center[0], object->ground_center[1],
                        object->ground_center[2]);
        tf2::Quaternion r2;
        r2.setRPY(0, 0, object->yaw_rad);
        tf2::Transform trans_obj(r2, v2);

        tf2::Transform v_out = trans_tf2 * trans_obj;
        tf2::Vector3 center_utm = v_out.getOrigin();

        // Right to left bound
        double kl = 23.5111796453;
        double bl = -1840954.14034;

        // Left to right bound
        double kr = 25.2669642857;
        double br = -2328797.75134;

        if ((center_utm.y() - bl) / kl < center_utm.x() &&
            (center_utm.y() - br) / kr > center_utm.x()) {
            temp_array.push_back(object);
        }
    }
    *objects_array = temp_array;
}