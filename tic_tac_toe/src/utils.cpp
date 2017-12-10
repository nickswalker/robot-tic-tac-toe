#include <geometry_msgs/PoseStamped.h>
#include "tic_tac_toe/utils.h"

double ready_pose[] = {
        0.295476645231,
       -0.326079112291,
       0.0612120516598,
        0.0154021102935,
       -0.999735891819,
       0.00489564239979,
       -0.0163363721222
};



geometry_msgs::PoseStamped array_to_pose(const double *array) {
    geometry_msgs::PoseStamped p_target;
    p_target.header.frame_id = "m1n6s200_link_base";
    p_target.pose.position.x = array[0];
    p_target.pose.position.y = array[1];
    p_target.pose.position.z = array[2];
    p_target.pose.orientation.x = array[3];
    p_target.pose.orientation.y = array[4];
    p_target.pose.orientation.z = array[5];
    p_target.pose.orientation.w = array[6];
    return p_target;
}

geometry_msgs::PoseStamped get_ready_pose() {
    return array_to_pose(ready_pose);
}
