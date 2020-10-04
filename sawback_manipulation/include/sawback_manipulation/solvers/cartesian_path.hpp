/**
 * @file cartesian_path.hpp
 * @author Boston Cleek
 * @date 4 Oct 2020
 * @brief Plan cartesian path using MoveIt's computeCartesianPath()
 * https://github.com/ros-planning/moveit/blob/master/moveit_core/robot_state/include/moveit/robot_state/cartesian_interpolator.h
 */

#pragma once

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/link_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/macros/class_forward.h>

#include <Eigen/Core>

namespace sawback_manipulation
{
namespace solvers
{
MOVEIT_CLASS_FORWARD(CartesianPath)

/** @brief Compose cartesian path */
class CartesianPath
{
public:
  CartesianPath();

  CartesianPath(double max_step, double jump_threshold, double min_fraction, double max_velocity_scaling_factor,
                double max_acceleration_scaling_factor);

  bool plan(moveit::core::RobotStatePtr& robot_start_state_ptr, robot_trajectory::RobotTrajectoryPtr& result,
            const moveit::core::RobotModelConstPtr& robot_model_ptr,
            const moveit::core::JointModelGroup* joint_model_group_ptr, const std::string& link,
            const Eigen::Vector3d direction, bool global_reference_frame, double distance);

private:
  double max_step_;
  double jump_threshold_;
  double min_fraction_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
};

}  // namespace solvers
}  // namespace sawback_manipulation
