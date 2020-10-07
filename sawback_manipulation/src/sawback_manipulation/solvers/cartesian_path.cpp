/**
 * @file cartesian_path.cpp
 * @author Boston Cleek
 * @date 4 Oct 2020
 * @brief Plan cartesian path using MoveIt's computeCartesianPath()
 */

#include <iostream>
#include <cmath>
#include <limits>

#include <ros/console.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <sawback_manipulation/solvers/cartesian_path.hpp>

namespace sawback_manipulation
{
namespace solvers
{
constexpr char LOGNAME[] = "CartesianPath";

CartesianPath::CartesianPath()
  : max_step_(0.001)
  , jump_threshold_(1.5)
  , min_fraction_(0.5)
  , max_velocity_scaling_factor_(1.0)
  , max_acceleration_scaling_factor_(1.0)
{
}

CartesianPath::CartesianPath(double max_step, double jump_threshold, double min_fraction,
                             double max_velocity_scaling_factor, double max_acceleration_scaling_factor)
  : max_step_(max_step)
  , jump_threshold_(jump_threshold)
  , min_fraction_(min_fraction)
  , max_velocity_scaling_factor_(max_velocity_scaling_factor)
  , max_acceleration_scaling_factor_(max_acceleration_scaling_factor)
{
}

bool CartesianPath::plan(moveit::core::RobotStatePtr& robot_start_state_ptr,
                         robot_trajectory::RobotTrajectoryPtr& result,
                         const moveit::core::JointModelGroup* joint_model_group_ptr, const std::string& link,
                         const Eigen::Vector3d& direction, bool global_reference_frame, double distance)
{
  // store trajectory from cartesian planner
  std::vector<moveit::core::RobotStatePtr> trajectory;
  // model link pointer
  const moveit::core::LinkModel* link_model_ptr = joint_model_group_ptr->getLinkModel(link);

  // Call MoveIt's computeCartesianPath()
  const double length_covered = moveit::core::CartesianInterpolator::computeCartesianPath(
      robot_start_state_ptr.get(), joint_model_group_ptr, trajectory, link_model_ptr, direction, global_reference_frame,
      distance, moveit::core::MaxEEFStep(max_step_), moveit::core::JumpThreshold(jump_threshold_));

  const double achieved_fraction = length_covered / distance;

  if (trajectory.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "The cartesian trajectory is empty");
    return false;
  }

  ROS_INFO_NAMED(LOGNAME, "Cartesain path coverage %f: ", achieved_fraction);

  // Copy trajectory to the result
  // This allows MoveIt to execute the result
  // result.reset(new robot_trajectory::RobotTrajectory(robot_model_ptr, joint_model_group_ptr));

  for (const auto& waypoint : trajectory)
  {
    // dt = 0 for now
    result->addSuffixWayPoint(waypoint, 0.0);
  }

  // Time scaling
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  timing.computeTimeStamps(*result, max_velocity_scaling_factor_, max_acceleration_scaling_factor_);

  // The fraction of the trajectory is greater than or equal to the min
  if (achieved_fraction > min_fraction_ ||
      std::fabs(achieved_fraction - min_fraction_) < std::numeric_limits<double>::epsilon())
  {
    return true;
  }

  return false;
}

}  // namespace solvers
}  // namespace sawback_manipulation
