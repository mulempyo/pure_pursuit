#include "pure_pursuit_planner/pure_pursuit_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)

namespace pure_pursuit_planner {

PurePursuitPlanner::PurePursuitPlanner()
  : is_initialized_(false), goal_reached_(false) {}

PurePursuitPlanner::PurePursuitPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : is_initialized_(false), goal_reached_(false) {
  initialize(name, tf, costmap_ros);
}

PurePursuitPlanner::~PurePursuitPlanner() {}

void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
  if (!is_initialized_) {
    
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    planner_util_.initialize(tf_, costmap, costmap_ros_->getGlobalFrameID());

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("lookahead_distance", lookahead_distance_, 0.5);
    private_nh.param("max_velocity", max_velocity_, 0.2);
    private_nh.param("min_velocity", min_velocity_, 0.1);  
    private_nh.param("max_angular_velocity", max_angular_velocity_, 1.5);
    private_nh.param("min_angular_velocity", min_angular_velocity_, 0.1);  
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);

    is_initialized_ = true;
    ROS_INFO("Pure Pursuit Planner initialized.");
  } else {
    ROS_WARN("Pure Pursuit Planner already initialized.");
  }
}

bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!is_initialized_) {
    ROS_ERROR("Pure Pursuit Planner has not been initialized.");
    return false;
  }
  ROS_WARN("start Plan");
  return planner_util_.setPlan(plan);
}

bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!is_initialized_) {
    ROS_ERROR("Pure Pursuit Planner has not been initialized.");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose.");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  transformed_plan.clear();
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
    ROS_ERROR("Could not get local plan");
    return false;
  }

   if (transformed_plan.empty()) {
    ROS_WARN("Received an empty transformed_plan.");
    return false;
  }

  global_plan_.resize(transformed_plan.size());
    for (unsigned int i = 0; i < transformed_plan.size(); ++i) {
      global_plan_[i] = transformed_plan[i];
    }

  geometry_msgs::PoseStamped lookahead_point = getLookaheadPoint(current_pose_,global_plan_);
  double yaw = getYaw(current_pose_);
  double target_yaw = atan2(lookahead_point.pose.position.y - current_pose_.pose.position.y,
                            lookahead_point.pose.position.x - current_pose_.pose.position.x);

  double yaw_error = angles::shortest_angular_distance(yaw, target_yaw);
  double distance_to_lookahead = getDistance(current_pose_, lookahead_point);

  double velocity_scale_factor = std::max(0.14, 1.0 - (fabs(yaw_error) / M_PI));
  double angular_scale_factor = 0.1;

  double adjusted_velocity = min_velocity_ + (max_velocity_ - min_velocity_) * velocity_scale_factor;
  double adjusted_angular_velocity = angular_scale_factor* max_angular_velocity_ * fabs(yaw_error / M_PI);

  cmd_vel.linear.x = adjusted_velocity;
  cmd_vel.angular.z = yaw_error < 0 ? -adjusted_angular_velocity : adjusted_angular_velocity;

  if (distance_to_lookahead < xy_goal_tolerance_ &&
      fabs(yaw_error) < yaw_goal_tolerance_) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_INFO("Goal reached.");
  }

  return true;
}

bool PurePursuitPlanner::isGoalReached() {
  return goal_reached_;
}

geometry_msgs::PoseStamped PurePursuitPlanner::getLookaheadPoint(const geometry_msgs::PoseStamped& current_pose_, std::vector<geometry_msgs::PoseStamped>& global_plan_) {
  double closest_distance = std::numeric_limits<double>::max();
  geometry_msgs::PoseStamped closest_point;
  for (const auto& pose : global_plan_) {
    double distance = getDistance(current_pose_, pose);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point = pose;
    }

    if (distance > lookahead_distance_) {
      break;
    }
  }
  return closest_point;
}

double PurePursuitPlanner::getDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
  return hypot(pose2.pose.position.x - pose1.pose.position.x,
               pose2.pose.position.y - pose1.pose.position.y);
}

double PurePursuitPlanner::getYaw(const geometry_msgs::PoseStamped& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace pure_pursuit_planner

