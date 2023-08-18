#include "PurePursuitPlanner.h"

namespace CUSTOM_PATH_PLANNER{


PurePursuitPlanner::PurePursuitPlanner(){

}
bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

    // getRobotPose（）
    // if ( ! costmap_ros_->getRobotPose(current_pose_)) {
    //   ROS_ERROR("Could not get robot pose");
    //   return false;
    // }
    // 全局路径裁剪
    // TODO 根据输入是全局路径还是相对路径进行转换

    //  相对路径
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

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

}

bool PurePursuitPlanner::isGoalReached(){


}

bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){


}

void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){


}


// 
geometry_msgs::PoseStamped PurePursuitPlanner::goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {

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