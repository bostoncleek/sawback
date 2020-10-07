/**
 * @file pick_place.hpp
 * @author Boston Cleek
 * @date 5 Oct 2020
 * @brief Pick and place task using MoveItcpp
 */

#pragma once

#include <vector>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseStamped.h>

#include <sawback_manipulation/solvers/cartesian_path.hpp>

// #include <Eigen/Core>

namespace sawback_manipulation
{
namespace tasks
{
MOVEIT_CLASS_FORWARD(PickPlace)

class PickPlace
{
public:
  PickPlace(const moveit::planning_interface::MoveItCppPtr& moveit_cpp, const std::string& arm_planning_group,
            const std::string& gripper_planning_group, const std::string& eef_link);

  // Assume pose in in base frame
  void initPose(double pre_distance, double post_distance, const Eigen::Isometry3d& pose);

  // void initPose(double pre_distance, double post_distance, const geometry_msgs::PoseStamped& pose);

  bool planPick();

  bool planPlace();

  bool execute();

  void displayFrames(const std::vector<std::pair<Eigen::Isometry3d, std::string>>& frames);

  void displayTrajectory();

private:
  bool planTo(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
              const Eigen::Isometry3d& goal_pose);

  bool planRelative(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
                    const Eigen::Vector3d& direction, bool root_frame, double distance);

  bool planGripper(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
                   const std::string& state);

private:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_ptr_;
  moveit::planning_interface::PlanningComponentPtr planning_component_arm_ptr_;
  moveit::planning_interface::PlanningComponentPtr planning_component_gripper_ptr_;

  std::string arm_planning_group_;      // planning group arm name
  std::string gripper_planning_group_;  // planning group gripper name
  std::string eef_link_;                // link name to plan the path for

  std::string root_link_;      // root link in urdf
  std::string arm_root_link_;  // arm root link in urdf
  // std::string action_;  // pick or place action

  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::JointModelGroupConstPtr joint_model_group_ptr_;  // only for arm

  std::vector<std::pair<robot_trajectory::RobotTrajectoryPtr, std::string>> trajectories_;

  sawback_manipulation::solvers::CartesianPathUniquePtr cartesian_path_ptr_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_ptr_;

  // const moveit::core::JointModelGroup* joint_model_group_ptr_;

  double pre_distance_;   // offset distance to pose
  double post_distance_;  // offset distance from pose
  // std::string pose_reference_frame_;  // frame of reference for pick or place
  Eigen::Isometry3d pose_;  // pick or place pose
  // geometry_msgs::PoseStamped pose_;

  Eigen::Isometry3d text_pose_;  // for rviz tools
};
}  // namespace tasks
}  // namespace sawback_manipulation
