/**
 * @file sawback_demo_node.cpp
 * @author Boston Cleek
 * @date 25 Sep 2020
 * @brief Demo showing capabilities of sawback using moveitcpp
 */

// C++
#include <iostream>
#include <memory>
#include <vector>

// // ROS
// #include <geometry_msgs/PointStamped.h>
// #include <moveit/moveit_cpp/moveit_cpp.h>
// #include <moveit/moveit_cpp/planning_component.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <ros/ros.h>

// #include <moveit/robot_state/cartesian_interpolator.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include <moveit/robot_trajectory/robot_trajectory.h>
// //
// #include <sawback_manipulation/solvers/cartesian_path.hpp>

#include <sawback_manipulation/tasks/pick_place.hpp>

constexpr char LOGNAME[] = "sawback_moveitcpp";

moveit_msgs::CollisionObject createObject()
{
  moveit_msgs::CollisionObject object;
  object.id = "box";
  object.header.frame_id = "base";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = 0.2;
  object.primitives[0].dimensions[1] = 0.2;
  object.primitives[0].dimensions[2] = 0.2;
  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = 0.7;
  object.primitive_poses[0].position.y = 0.0;
  object.primitive_poses[0].position.z = -0.2;
  object.primitive_poses[0].orientation.w = 1.0;
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sawback_moveitcpp");
  ros::NodeHandle nh("/sawback_moveitcpp");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting...");

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  moveit::planning_interface::PlanningComponentPtr planning_component_ptr =
      std::make_shared<moveit::planning_interface::PlanningComponent>("right_arm", moveit_cpp_ptr);

  // TODO: add floor and object
  // const moveit_msgs::CollisionObject box = createObject();

  // starting configuration
  const double initial_joints[] = { 0.9389267787249193,  -0.2050500749605062, -1.0421734917046992, 1.4588063551704888,
                                    -1.8597856835303392, -1.0772340009395818, -0.22431043800663522 };

  // move to starting configuration
  const moveit::core::RobotModelConstPtr robot_model =
      std::const_pointer_cast<const moveit::core::RobotModel>(moveit_cpp_ptr->getRobotModel());

  moveit::core::RobotStatePtr robot_state_ptr = make_shared<moveit::core::RobotState>(robot_model);

  const moveit::core::JointModelGroup* joint_model_group_ptr = robot_state_ptr->getJointModelGroup("right_arm");

  // Set joint angles to positions from IK
  robot_state_ptr->setJointGroupPositions(joint_model_group_ptr, initial_joints);
  // Update all the transforms
  robot_state_ptr->update(initial_joints);

  planning_component_ptr->setStartStateToCurrentState();
  planning_component_ptr->setGoal(const_cast<const robot_state::RobotState&>(*robot_state_ptr.get()));

  const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_ptr->plan();

  if (plan.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    moveit_cpp_ptr->execute("right_arm", plan.trajectory);
  }

  else
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Failed to move to initial configuration");
    ros::shutdown();
    return 0;
  }

  // Begin manipulation tasks
  auto task = std::make_unique<sawback_manipulation::tasks::PickPlace>(moveit_cpp_ptr, "right_arm", "hand",
                                                                       "right_gripper_tip");
  // const Eigen::Isometry3d goal =
  //     Eigen::Translation3d(0.7, 0.0303, 0.6069) * Eigen::Quaterniond(0.696, 0.123, 0.696, 0.123);

  const double roll = 0.0, pitch = 1.5708 * 2.0, yaw = 1.5708 * 2.0;
  const Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  const Eigen::Isometry3d goal = Eigen::Translation3d(0.8, 0.1, -0.25) * quat;

  task->initPick(0.15, 0.15, goal);
  task->planPick();

  // task->initPlace(0.15, 0.15, goal);
  // task->planPlace();

  // task->execute();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
  return 0;
}

//
