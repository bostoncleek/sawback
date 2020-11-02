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

#include <sawback_manipulation/cartesian_path.hpp>

namespace sawback_manipulation
{
MOVEIT_CLASS_FORWARD(PickPlace)

/** @brief Pick and place pipeline */
class PickPlace
{
public:
  /**
   * @brief Constructor
   * @param moveit_cpp - moveitcpp
   * @param arm_planning_group - arm planning group name
   * @param gripper_planning_group - gripper planning group name
   * @param eef_link - end effector link name
   */
  PickPlace(const moveit::planning_interface::MoveItCppPtr& moveit_cpp, const std::string& arm_planning_group,
            const std::string& gripper_planning_group, const std::string& eef_link);

  /**
   * @brief Initialize a pick task
   * @param pre_distance - approach distance from grasp pose
   * @param post_distance - retreat distance from grasp pose
   * @param pose - grasp pose in base frame of sawyer
   * @details Assumes the grasp pose approach direction is the x-axis
   */
  void initPick(double pre_distance, double post_distance, const Eigen::Isometry3d& pose);

  /**
   * @brief Initialize a place task
   * @param pre_distance - approach distance from place pose
   * @param post_distance - retreat distance from place pose
   * @param pose - place pose in base frame of sawyer
   * @details Assumes the place pose approach direction is the x-axis
   */
  void initPlace(double pre_distance, double post_distance, const Eigen::Isometry3d& pose);

  /**
   * @brief Plans and executed the pick task
   * @return true if planning and execution are successful
   */
  bool pick();

  /**
   * @brief Plans and executed the pick task
   * @return true if planning and execution are successful
   */
  bool place();

  /**
   * @brief Displays frames in base_link frame
   * @param frames - frame pose and name
   */
  void displayFrames(const std::vector<std::pair<Eigen::Isometry3d, std::string>>& frames);

  /**
   * @brief Displays trajectory in base_link frame
   * @param trajectory - planned trajectory
   */
  void displayTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory);

private:
  /**
   * @brief Plan path to a pose goal
   * @param result - planned trajectory
   * @param start_state - robot start state
   * @param goal_pose - pose goal
   * @return true if planning is successful
   */
  bool planTo(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
              const Eigen::Isometry3d& goal_pose);

  /**
   * @brief Cartesian path planning to a pose goal
   * @param result - planned trajectory
   * @param start_state - robot start state
   * @param planning_scene_monitor_ptr - planning scene monitor
   * @param direction - unit vector direction of travel
   * @param root_frame - root URDF frame for planning
   * @param distance - distance to travel in the specified direction
   * @return true if planning is successful
   */
  bool planRelative(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
                    const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor_ptr,
                    const Eigen::Vector3d& direction, bool root_frame, double distance);

  /**
   * @brief Plan gripper opening and closing path
   * @param result - planned trajectory
   * @param start_state - robot start state
   * @param state - open/close
   * @return true if planning is successful
   */
  bool planGripper(robot_trajectory::RobotTrajectoryPtr& result, const moveit::core::RobotStateConstPtr& start_state,
                   const std::string& state);

private:
  moveit::planning_interface::MoveItCppPtr moveit_cpp_ptr_;                          // moveitcpp
  moveit::planning_interface::PlanningComponentPtr planning_component_arm_ptr_;      // arm planning component
  moveit::planning_interface::PlanningComponentPtr planning_component_gripper_ptr_;  // gripper planning component

  std::string arm_planning_group_;      // planning group arm name
  std::string gripper_planning_group_;  // planning group gripper name
  std::string eef_link_;                // link name to plan the path for

  std::string root_link_;      // root link in urdf
  std::string arm_root_link_;  // arm root link in urdf

  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::JointModelGroupConstPtr joint_model_group_ptr_;  // only for arm

  sawback_manipulation::CartesianPathUniquePtr cartesian_path_ptr_;           // cartesian planning warpper
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_ptr_;  // visualization

  double pre_distance_pick_;     // offset distance to pick pose
  double post_distance_pick_;    // offset distance from pick pose
  Eigen::Isometry3d pick_pose_;  // pick pose

  double pre_distance_place_;     // offset distance to place pose
  double post_distance_place_;    // offset distance from place pose
  Eigen::Isometry3d place_pose_;  // place pose

  Eigen::Isometry3d text_pose_;  // for rviz tools
};
}  // namespace sawback_manipulation
