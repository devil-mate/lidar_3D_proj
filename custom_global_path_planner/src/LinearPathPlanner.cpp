

#include <pluginlib/class_list_macros.hpp>
#include "LinearPathPlanner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(custom_global_path_planner::LinearPathPlanner, nav_core::BaseGlobalPlanner);

namespace custom_global_path_planner{

LinearPathPlanner::LinearPathPlanner(){

}
LinearPathPlanner::~LinearPathPlanner(){

}
void LinearPathPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    
}


bool LinearPathPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    ROS_INFO("make plan start:[%f %f], goal:[%f %f]", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    plan.clear();
    float goal_distance = sqrt(pow((start.pose.position.x - goal.pose.position.x), 2) + pow((start.pose.position.y - goal.pose.position.y), 2));
    float yaw = atan2(goal.pose.position.y - start.pose.position.y, goal.pose.position.x - start.pose.position.x);
    ROS_DEBUG("goal distance: %f, yaw: %f", goal_distance, yaw);

    float delta = 0.1; // 间隔delta输出start至end的直线上的点 我们间隔0.1取直线上的所有点，放到输出的参数plan里
    int n = 0; //每个插值点的索引
    while (n * delta < goal_distance){
        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = (n * delta) * cos(yaw) + start.pose.position.x;
        pose.pose.position.y = (n * delta) * sin(yaw) + start.pose.position.y;
        ++n;
        plan.push_back(pose);
    }
    plan.push_back(goal); // 终点
    return !plan.empty();

}



}