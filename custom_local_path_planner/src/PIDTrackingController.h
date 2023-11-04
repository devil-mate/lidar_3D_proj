#ifndef _PID_TRACKING_CONTROLLER_H
#define _PID_TRACKING_CONTROLLER_H

#include <ros/ros.h>
#include "PIDController.h"
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
    ros::NodeHandle n;
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg);
    bool initialized_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;

    std::string global_frame_;  ///< @brief The global frame for the costmap
    std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
    std::shared_ptr<PIDControllerInterface> pidController_;
};

}
#endif

