#ifndef _PURE_PURSUIT_PLANNER_H
#define _PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>

#include "nav_core/base_global_planner.h"
#include "geometry_msgs/Twist.h"


namespace CUSTOM_PATH_PLANNER{


class PurePursuitPlanner: public nav_core::BaseGlobalPlanner{

public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
private:
    geometry_msgs::PoseStamped goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg);


};

}
#endif

