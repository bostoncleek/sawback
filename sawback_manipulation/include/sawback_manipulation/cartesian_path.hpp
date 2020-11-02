/**
 * @file cartesian_path.hpp
 * @author Boston Cleek
 * @date 4 Oct 2020
 * @brief Plan cartesian path using MoveIt's computeCartesianPath()
 */

#pragma once

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/kinematic_constraints/kinematic_constraint.h>

#include <Eigen/Core>

namespace sawback_manipulation
{
MOVEIT_CLASS_FORWARD(CartesianPath)

/** @brief Compose cartesian path */
class CartesianPath
{
public:
  /** @brief Constructor sets parameters to default values */
  CartesianPath();

  /**
   * @brief Constructor
   * @param max_step - max step size in path for translation
   * @param jump_threshold - threshold in joint angles
   * @param min_fraction - fraction of path planned to be considered a success
   * @param max_velocity_scaling_factor - joint velocity scaling factor [0,1]
   * @param max_acceleration_scaling_factor - joint acceleration scaling factor [0,1]
   */
  CartesianPath(double max_step, double jump_threshold, double min_fraction, double max_velocity_scaling_factor,
                double max_acceleration_scaling_factor);

  /**
   * @brief Plan a cartesian path from a start state to a goal state
   * @param robot_model_ptr - robot model std::shared_ptr<const T>
   * @param joint_model_group_ptr - joint model group raw pointer
   * @param link - link name to plan the path for
   * @param direction - unit vector for path direction
   * @param global_reference_frame - true if planning is in global frame else for planning in link's frame
   * @param distance - distance to move in the direction inidicated
   * @param robot_start_state_ptr [out] - starting robot state std::shared_ptr<T>
   * @param result [out] - planned trajectory std::shared_ptr<T>
   * @return true is the fraction of the path planned is greater than or equal to the min fraction
   * @details Uses MoveIt's computeCartesianPath() to plan a path between two states
   */
  bool plan(moveit::core::RobotStatePtr& robot_start_state_ptr, robot_trajectory::RobotTrajectoryPtr& result,
            const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor_ptr,
            const moveit::core::JointModelGroup* joint_model_group_ptr, const std::string& link,
            const Eigen::Vector3d& direction, bool global_reference_frame, double distance);

private:
  bool isStateValid(const planning_scene::PlanningScene* planning_scene, moveit::core::RobotState* robot_state,
                    const moveit::core::JointModelGroup* joint_model_group, const double* joint_positions);

private:
  double max_step_;                         // max step size in path for translation
  double jump_threshold_;                   // threshold in joint angles
  double min_fraction_;                     // fraction of path planned to be considered a success
  double max_velocity_scaling_factor_;      // joint velocity scaling factor [0,1]
  double max_acceleration_scaling_factor_;  // joint acceleration scaling factor [0,1]
};

}  // namespace sawback_manipulation
