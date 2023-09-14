#ifndef _LINEAR_PATH_PLANNER_H
#define _LINEAR_PATH_PLANNER_H
#include <ros/ros.h>

#include <nav_core/base_global_planner.h>

namespace custom_global_path_planner{

class LinearPathPlanner : public nav_core::BaseGlobalPlanner {
public:
    LinearPathPlanner();    
    ~LinearPathPlanner(); 
       
    bool makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override; 

private:


};



}



#endif