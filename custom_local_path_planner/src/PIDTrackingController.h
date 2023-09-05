#ifndef _PID_TRACKING_CONTROLLER_H
#define _PID_TRACKING_CONTROLLER_H

#include <ros/ros.h>

#include "nav_core/base_local_planner.h"
#include "geometry_msgs/Twist.h"


namespace CUSTOM_PATH_PLANNER{


class PIDTrackingController: public nav_core::BaseLocalPlanner{

public:
    PIDTrackingController();
    ~PIDTrackingController();
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
private:
    geometry_msgs::PoseStamped goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
    bool initialized_;

};

}
#endif

