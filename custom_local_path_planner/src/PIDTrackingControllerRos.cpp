#include "PIDTrackingControllerRos.h"

namespace CUSTOM_PATH_PLANNER{


PIDTrackingControllerRos::PIDTrackingControllerRos(){

}
bool PIDTrackingControllerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

  if(!initialized_){
    ROS_ERROR("local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
    

    // Get robot velocity
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    if ( !getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    // 全局路径裁剪
    // TODO 根据输入是全局路径还是相对路径进行转换
    pruneGlobalPlan();
    // 
    if (checkGoalReached()){
        // STOP cmd_vel=0
        return true;
    }
    //  相对路径
    // std::vector<geometry_msgs::PoseStamped> transformed_plan;
    // if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    //   ROS_ERROR("Could not get local plan");
    //   return false;
    // }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  

}

// 
bool PIDTrackingControllerRos::checkGoalReached(){
    // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
        || cfg_.goal_tolerance.free_goal_vel))
  {
    goal_reached_ = true;
    return true;
  }
  return false;
}

bool PIDTrackingControllerRos::isGoalReached(){
    if (goal_reached_){
        ROS_INFO("GOAL Reached!");
        planner_->clearPlanner();
        // return true;
    }
    return goal_reached_;
    // return false;
    }

}

bool PIDTrackingControllerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){


}

void PIDTrackingControllerRos::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){


    initialized_ = true;
}


// 
geometry_msgs::PoseStamped PIDTrackingControllerRos::goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {

// #if ROS_VERSION_GE(ROS_VERSION_MAJOR, ROS_VERSION_MINOR, ROS_VERSION_PATCH, 1, 14, 0)

//   geometry_msgs::PoseStamped goal_pose, base_pose_msg;

//   goal_pose = goal_pose_msg;

//   goal_pose.header.stamp = ros::Time(0.0);

//   try {

//     base_pose_msg = tf_->transform(goal_pose, "base_link");

//   } catch (tf2::TransformException& ex) {

//     ROS_WARN("transform err");

//     return base_pose_msg;

//   }

// #else

  geometry_msgs::PoseStamped base_pose_msg;

  tf::Stamped<tf::Pose> goal_pose, base_pose;

  poseStampedMsgToTF(goal_pose_msg, goal_pose);

  goal_pose.stamp_ = ros::Time();

  try {

    tf_->transformPose(costmap_ros_->getBaseFrameID(), goal_pose, base_pose);

  } catch (tf::TransformException& ex) {

    ROS_WARN("transform err");

    return base_pose_msg;

  }

  tf::poseStampedTFToMsg(base_pose, base_pose_msg);

// #endif

  return base_pose_msg;

}


}

bool PIDTrackingControllerRos::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
