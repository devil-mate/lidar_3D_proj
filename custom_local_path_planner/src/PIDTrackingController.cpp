#include "PIDTrackingController.h"

namespace CUSTOM_PATH_PLANNER{


PIDTrackingController::PIDTrackingController():nh("~"){
    pidController_ = std::make_pair<PIDController>();
    

}
void PIDTrackingController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){


    initialized_ = true;
}
void PIDTrackingController::initParams(){

    try
    {
        nh.param("global_frame", global_frame_, std::string("odom"));
        nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

        pidController_->params_.Kd=1.0;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM(e.what() );
        throw -1;
    }


}
bool PIDTrackingController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

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
    // pruneGlobalPlan();
    //  TEST/TEMP 不裁剪，直接跟踪全局路径

    //  相对路径
    // std::vector<geometry_msgs::PoseStamped> transformed_plan;
    // if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    //   ROS_ERROR("Could not get local plan");
    //   return false;
    // }

    //if the global plan passed in is empty... we won't do anything
    // TODO 坐标变换
    // 
    if(global_plan_.empty()) {
      ROS_WARN_NAMED("local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("local_planner", "Received a transformed plan with %zu points.", global_plan_.size());

    


    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    // dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

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
        bool isOk = PIDComputeVelocityCommands(current_pose_, cmd_vel);
            
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

bool PIDTrackingController::PIDComputeVelocityCommands(geometry_msgs::PoseStamped &currentPose, geometry_msgs::Twist& cmd_vel){
    geometry_msgs::PoseStamped previewPose;
    calcPreviewPoint(previewPose);
    // TODO dt
    float yaw =pidController_->calculate(previewPose,current_pose_,dt);
    // TODO 
    // （yaw-yaw_current）/dt
    double yaw_current= tf::getYaw(currentPose.pose.orientation);
    double w =(yaw -yaw_current);
    cmd_vel.angular.z=(yaw-yaw_current)/dt;

}
bool PIDTrackingController::calcPreviewPoint(geometry_msgs::PoseStamped &previewPose){


}
// 获取机器人位姿
bool  PIDTrackingController::getRobotPose(geometry_msgs::PoseStamped &global_pose){
tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    // use current time if possible (makes sure it's not in the future)
    if (tf_.canTransform(global_frame_, robot_base_frame_, current_time))
    {
      geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, robot_base_frame_, current_time);
      tf2::doTransform(robot_pose, global_pose, transform);
    }
    // use the latest otherwise
    else
    {
      tf_.transform(robot_pose, global_pose, global_frame_);
    }
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

bool PIDTrackingController::isGoalReached(){


}

bool PIDTrackingController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){

  if(!initialized_){
    ROS_ERROR("local planner  have not been initialized, please call initialize() first");
    return false;
  }
    global_plan_.clear();
    global_plan_ = plan;
    
    return true;

}



// 
geometry_msgs::PoseStamped PIDTrackingController::goalToBaseFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {

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