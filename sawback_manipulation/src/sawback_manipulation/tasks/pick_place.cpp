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

PickPlace::PickPlace(const moveit::planning_interface::MoveItCppPtr& moveit_cpp, const std::string& arm_planning_group,
                     const std::string& gripper_planning_group, const std::string& eef_link)
  : moveit_cpp_ptr_(moveit_cpp)
  , arm_planning_group_(arm_planning_group)
  , gripper_planning_group_(gripper_planning_group)
  , eef_link_(eef_link)
  , arm_root_link_("base")
{
  // TODO: read cartesian path parameters

  planning_component_arm_ptr_.reset(
      new moveit::planning_interface::PlanningComponent(arm_planning_group_, moveit_cpp_ptr_));
  planning_component_gripper_ptr_.reset(
      new moveit::planning_interface::PlanningComponent(gripper_planning_group, moveit_cpp_ptr_));

  robot_model_ptr_.reset(moveit_cpp_ptr_->getRobotModel().get());
  joint_model_group_ptr_.reset(robot_model_ptr_->getJointModelGroup(arm_planning_group_));

  // URDF root link
  root_link_ = robot_model_ptr_->getRootLinkName();
  ROS_INFO_NAMED(LOGNAME, "URDF root link %s: ", root_link_.c_str());

  cartesian_path_ptr_.reset(new sawback_manipulation::solvers::CartesianPath);

  // Visualization
  visual_tools_ptr_.reset(new moveit_visual_tools::MoveItVisualTools(root_link_, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                     moveit_cpp_ptr_->getPlanningSceneMonitor()));

  visual_tools_ptr_->deleteAllMarkers();
  visual_tools_ptr_->loadRemoteControl();

  Eigen::Isometry3d text_pose_ = Eigen::Isometry3d::Identity();
  text_pose_.translation().z() = 0.5;

  // visual_tools_ptr_->publishText(text_pose_, "sawback manipulation", rviz_visual_tools::WHITE,
  //                                rviz_visual_tools::XLARGE);
  // visual_tools_ptr_->trigger();
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
  // TODO: fix cartesian path fraction results

  // clear previous trajectory
  trajectories_.clear();

  // Compose all the frame transformations
  // Move in z-axis for cartesian motion but frame may change
  Eigen::Vector3d direction(0.0, 0.0, 1.0);

  // Transform from urdf root link to arm root link
  moveit::core::RobotStatePtr robot_start_state_ptr = moveit_cpp_ptr_->getCurrentState();
  const Eigen::Isometry3d Troot_base = robot_start_state_ptr->getGlobalLinkTransform(arm_root_link_);

  // Transform from urdf root link to end effect
  const Eigen::Isometry3d start_pose = robot_start_state_ptr->getGlobalLinkTransform(eef_link_);

  // Inverse of the transform from the pre grasp to the grasp in the frame of the gripper
  const Eigen::Isometry3d Tpre_grasp_inv =
      Eigen::Translation3d(0.0, 0.0, -pre_distance_) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Transform from urdf root link to grasp
  const Eigen::Isometry3d grasp_pose = Troot_base * pose_;

  // Transform from urdf root link to the pre-grasp
  const Eigen::Isometry3d pre_grasp_pose = grasp_pose * Tpre_grasp_inv;

  // Transform from the grasp pose to the post grasp in the root frame
  Eigen::Isometry3d Tgrasp_post =
      Eigen::Translation3d(0.0, 0.0, post_distance_) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Translation in frame of grasp
  Tgrasp_post.translation() = grasp_pose.inverse().rotation() * Tgrasp_post.translation();

  const Eigen::Isometry3d post_grasp_pose = grasp_pose * Tgrasp_post;

  const std::vector<std::pair<Eigen::Isometry3d, std::string>> frames = {
    std::make_pair(start_pose, "Start pose"), std::make_pair(pre_grasp_pose, "Pre-grasp"),
    std::make_pair(grasp_pose, "Grasp"), std::make_pair(post_grasp_pose, "Post-grasp")
  };
  // Visualize planning frames
  displayFrames(frames);

  //////////////////////////////////////////////////////////////////////////
  // 1) Move to offset
  auto trajectory_pre_grasp =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, arm_planning_group_);

  const moveit::core::RobotStateConstPtr robot_start_state =
      std::const_pointer_cast<const moveit::core::RobotState>(robot_start_state_ptr);

  // std::cout << "------------" << std::endl;
  // std::cout << " Start " << std::endl;
  // robot_start_state->printStatePositions();
  // std::cout << "------------" << std::endl;

  if (!planTo(trajectory_pre_grasp, robot_start_state, pre_grasp_pose))
  {
    return false;
  }

  trajectories_.emplace_back(std::make_pair(trajectory_pre_grasp, arm_planning_group_));

  //////////////////////////////////////////////////////////////////////////
  // 2) Attach object from scene, disable collisions, and open gripper
  // TODO: add objects to planning scene

  auto trajectory_open = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, gripper_planning_group_);

  const moveit::core::RobotStateConstPtr pre_gripper_open_state =
      std::const_pointer_cast<const moveit::core::RobotState>(trajectory_pre_grasp->getLastWayPointPtr());

  // std::cout << "------------" << std::endl;
  // std::cout << " Pre cartesian 1 (before gripper opens) " << std::endl;
  // pre_gripper_open_state->printStatePositions();
  // std::cout << "------------" << std::endl;

  if (!planGripper(trajectory_open, pre_gripper_open_state, "open"))
  {
    return false;
  }

  trajectories_.emplace_back(std::make_pair(trajectory_open, gripper_planning_group_));

  //////////////////////////////////////////////////////////////////////////
  // 3) Move relative to grasp in z-axis of end-effector
  auto trajectory_grasp = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, arm_planning_group_);

  const moveit::core::RobotStateConstPtr pre_grasp_state =
      std::const_pointer_cast<const moveit::core::RobotState>(trajectory_open->getLastWayPointPtr());

  // std::cout << "------------" << std::endl;
  // std::cout << " Pre cartesian 1 (gripper open) " << std::endl;
  // pre_grasp_state->printStatePositions();
  // std::cout << "------------" << std::endl;

  if (!planRelative(trajectory_grasp, pre_grasp_state, direction, false, pre_distance_))
  {
    return false;
  }

  ROS_WARN_NAMED(LOGNAME, "Approach Length of trajectory %lu: ", trajectory_grasp->getWayPointCount());

  trajectories_.emplace_back(std::make_pair(trajectory_grasp, arm_planning_group_));

  //////////////////////////////////////////////////////////////////////////
  // 4) Attach object from scene, disable collisions, and close gripper
  // TODO: add objects to planning scene

  auto trajectory_close =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, gripper_planning_group_);

  const moveit::core::RobotStateConstPtr pre_gripper_close_state =
      std::const_pointer_cast<const moveit::core::RobotState>(trajectory_grasp->getLastWayPointPtr());

  // std::cout << "------------" << std::endl;
  // std::cout << " Completed cartesian 1 (gripper open) " << std::endl;
  // pre_gripper_close_state->printStatePositions();
  // std::cout << "------------" << std::endl;

  if (!planGripper(trajectory_close, pre_gripper_close_state, "close"))
  {
    return false;
  }

  trajectories_.emplace_back(std::make_pair(trajectory_close, gripper_planning_group_));

  //////////////////////////////////////////////////////////////////////////
  // 5) Move relative to grasp in z-axis of root link urdf frame
  auto trajectory_post_grasp =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_ptr_, arm_planning_group_);

  const moveit::core::RobotStateConstPtr grasp_state =
      std::const_pointer_cast<const moveit::core::RobotState>(trajectory_close->getLastWayPointPtr());

  // std::cout << "------------" << std::endl;
  // std::cout << " Pre cartesian 2 (gripper close) " << std::endl;
  // grasp_state->printStatePositions();
  // std::cout << "------------" << std::endl;

  if (!planRelative(trajectory_post_grasp, grasp_state, direction, true, post_distance_))
  {
    return false;
  }

  ROS_WARN_NAMED(LOGNAME, "Retreat Length of trajectory %lu: ", trajectory_post_grasp->getWayPointCount());

  trajectories_.emplace_back(std::make_pair(trajectory_post_grasp, arm_planning_group_));

  // Visualize plan
  displayTrajectory();

  return true;
}

bool PickPlace::planPlace()
{
}

bool PickPlace::execute()
{
  int i = 1;
  for (const auto& trajectory : trajectories_)
  {
    std::cout << "Traj: " << i << std::endl;
    if (trajectory.second == arm_planning_group_)
    {
      moveit_cpp_ptr_->execute(arm_planning_group_, trajectory.first);
    }
    else
    {
      moveit_cpp_ptr_->execute(gripper_planning_group_, trajectory.first);
    }
    i++;
  }
}

void PickPlace::displayFrames(const std::vector<std::pair<Eigen::Isometry3d, std::string>>& frames)
{
  for (const auto& frame : frames)
  {
    visual_tools_ptr_->publishAxisLabeled(frame.first, frame.second);
    // visual_tools_ptr_->publishText(text_pose_, frame.second, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }
  visual_tools_ptr_->trigger();
}

void PickPlace::displayTrajectory()
{
  for (const auto& trajectory : trajectories_)
  {
    if (trajectory.second == arm_planning_group_)
    {
      visual_tools_ptr_->publishTrajectoryLine(trajectory.first, joint_model_group_ptr_.get());
    }
  }
  visual_tools_ptr_->trigger();
}

bool PickPlace::planTo(robot_trajectory::RobotTrajectoryPtr& result,
                       const moveit::core::RobotStateConstPtr& start_state, const Eigen::Isometry3d& goal_pose)
{
  // TODO: Sent in starting robot state
  // TODO: check moveit_msgs::MoveItErrorCodes::SUCCESS

  // Convert from Eigen to poseStamped
  // Extract orientation
  const Eigen::Quaterniond quat(goal_pose.rotation());

  geometry_msgs::PoseStamped target;
  target.header.frame_id = root_link_;
  target.pose.position.x = goal_pose.translation().x();
  target.pose.position.y = goal_pose.translation().y();
  target.pose.position.z = goal_pose.translation().z();
  target.pose.orientation.w = quat.w();
  target.pose.orientation.x = quat.x();
  target.pose.orientation.y = quat.y();
  target.pose.orientation.z = quat.z();

  // Plan from current state
  planning_component_arm_ptr_->setStartState(*start_state.get());

  // Plan
  planning_component_arm_ptr_->setGoal(target, eef_link_);
  const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_arm_ptr_->plan();

  // if (!plan.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  if (!plan)
  {
    ROS_ERROR_NAMED(LOGNAME, "Planning failed");
    return false;
  }

  result = plan.trajectory;
  return true;
}

bool PickPlace::planRelative(robot_trajectory::RobotTrajectoryPtr& result,
                             const moveit::core::RobotStateConstPtr& start_state, const Eigen::Vector3d& direction,
                             bool root_frame, double distance)
{
  auto robot_start_state_ptr = std::make_shared<moveit::core::RobotState>(*start_state.get());

  if (!cartesian_path_ptr_->plan(robot_start_state_ptr, result, joint_model_group_ptr_.get(), eef_link_, direction,
                                 root_frame, distance))
  {
    ROS_ERROR_NAMED(LOGNAME, "Cartesian planning failed");
    return false;
  }

  return true;
}

bool PickPlace::planGripper(robot_trajectory::RobotTrajectoryPtr& result,
                            const moveit::core::RobotStateConstPtr& start_state, const std::string& state)
{
  planning_component_gripper_ptr_->setStartState(*start_state.get());
  planning_component_gripper_ptr_->setGoal(state);

  const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_gripper_ptr_->plan();

  // if (!plan.error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  if (!plan)
  {
    ROS_ERROR_NAMED(LOGNAME, "Gripper planning failed");
    return false;
  }

  result = plan.trajectory;
  return true;
}

}  // namespace tasks
}  // namespace sawback_manipulation

//
