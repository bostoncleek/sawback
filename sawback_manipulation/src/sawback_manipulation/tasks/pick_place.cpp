/**
 * @file pick_place.cpp
 * @author Boston Cleek
 * @date 5 Oct 2020
 * @brief Pick and place task using MoveItcpp
 */

#include <ros/console.h>

#include <sawback_manipulation/tasks/pick_place.hpp>

#include <Eigen/Geometry>

namespace sawback_manipulation
{
namespace tasks
{
constexpr char LOGNAME[] = "PickPlace";

PickPlace::PickPlace(const moveit::planning_interface::MoveItCppPtr& moveit_cpp, const std::string& planning_group,
                     const std::string& eef_link)
  : moveit_cpp_ptr_(moveit_cpp), planning_group_(planning_group), eef_link_(eef_link)
{
  planning_component_ptr_.reset(new moveit::planning_interface::PlanningComponent(planning_group_, moveit_cpp_ptr_));

  robot_model_ptr_.reset(moveit_cpp_ptr_->getRobotModel().get());
  joint_model_group_ptr_.reset(robot_model_ptr_->getJointModelGroup(planning_group_));
}

void PickPlace::initPose(double pre_distance, double post_distance, const Eigen::Isometry3d& pose)
{
  pre_distance_ = pre_distance;
  post_distance_ = post_distance;
  pose_ = pose;
}

// void PickPlace::initPose(double pre_distance, double post_distance, const geometry_msgs::PoseStamped& pose);
// {
//   pre_distance_ = pre_distance;
//   post_distance_ = post_distance;
//   pose_ = pose;
// }

bool PickPlace::planPick()
{
  // TODO: verify initPose has been set

  // 1) Move to offset
  // Inverse of the transform from the pre grasp to the grasp in the frame of the gripper
  const Eigen::Isometry3d Tinv =
      Eigen::Translation3d(0.0, 0.0, -pre_distance_) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Transform from sawyer base to the pre-grasp
  const Eigen::Isometry3d pre_grasp_pose = pose_ * Tinv;

  auto trajectory_pre_grasp = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, planning_group_);

  if(!planTo(trajectory_pre_grasp, pre_grasp_pose))
  {
    return;
  }
}

bool PickPlace::planPlace()
{
}

bool PickPlace::execute()
{
}

void PickPlace::offsetPose()
{
}

bool PickPlace::planTo(robot_trajectory::RobotTrajectoryPtr& result, const Eigen::Isometry3d& goal_pose)
{
  // TODO: send goal in frame of base_link so visualization looks correct in rviz

  // Convert from Eigen to poseStamped
  // Extract orientation
  const Eigen::Quaterniond quat(goal_pose.rotation());

  geometry_msgs::PoseStamped target;
  target.header.frame_id = "base";
  target.pose.position.x = goal_pose.translation().x();
  target.pose.position.y = goal_pose.translation().y();
  target.pose.position.z = goal_pose.translation().y();
  target.pose.orientation.w = quat.w();
  target.pose.orientation.x = quat.x();
  target.pose.orientation.y = quat.y();
  target.pose.orientation.z = quat.z();

  // Plan from current state
  planning_component_ptr_->setStartStateToCurrentState();

  // Plan
  planning_component_ptr_->setGoal(target, eef_link_);
  moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_ptr_->plan();

  if (!plan)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }

  result = plan.trajectory;
  return true;
}

bool PickPlace::planRelative(robot_trajectory::RobotTrajectoryPtr& result, bool eef_frame)
{
}

}  // namespace tasks
}  // namespace sawback_manipulation

//
