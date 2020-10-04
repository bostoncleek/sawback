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
#include <geometry_msgs/PointStamped.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
//
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
//
#include <sawback_manipulation/solvers/cartesian_path.hpp>

static const std::string PLANNING_GROUP = "right_arm";
static const std::string LOGNAME = "sawback_moveitcpp";

namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sawback_moveitcpp");
  ros::NodeHandle nh("/sawback_moveitcpp");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting...");

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components =
      std::make_shared<moveit::planning_interface::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);

  moveit::core::RobotModelConstPtr robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  moveit::core::RobotStatePtr robot_state_ptr = planning_components->getStartState();
  const moveit::core::JointModelGroup* joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.75;
  visual_tools.publishText(text_pose, "Sawback Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  planning_components->setStartStateToCurrentState();

  // robot_state_ptr->printStateInfo();
  // robot_state_ptr->printStatePositions();

  visual_tools.publishAxisLabeled(robot_state_ptr->getGlobalLinkTransform("right_hand"), "start_pose");
  visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // ROS_INFO_STREAM_NAMED("Root link: %s",
  // robot_model_ptr->getRootLinkName().c_str()); ROS_INFO_STREAM_NAMED("Model
  // frame: %s", robot_model_ptr->getModelFrame ().c_str());
  //
  // std::cout << robot_state_ptr->getGlobalLinkTransform("right_hand").matrix()
  // << std::endl;

  // bool frame_found = false;
  // bool *frame_found_ptr = &frame_found;
  // std::cout << robot_state_ptr->getFrameTransform("right_hand",
  // frame_found_ptr).matrix() << std::endl; std::cout << "Frame found: " <<
  // frame_found << std::endl;

  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "base_link";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.8;
  target_pose1.pose.position.y = 0.1603;
  target_pose1.pose.position.z = 0.6069;

  planning_components->setGoal(target_pose1, "right_hand");

  auto plan_solution1 = planning_components->plan();

  if (plan_solution1)
  {
    visual_tools.publishAxisLabeled(robot_state_ptr->getGlobalLinkTransform("right_hand"), "start_pose");
    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the goal pose in Rviz
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the trajectory in Rviz
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
    visual_tools.trigger();

    // ROS_INFO_STREAM_NAMED(LOGNAME, "Motion plan found");
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    planning_components->execute();
  }

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to "
                      "perfrom cartesian planning");

  planning_components->setStartStateToCurrentState();
  robot_state_ptr = moveit_cpp_ptr->getCurrentState();

  Eigen::Vector3d direction;
  direction.x() = 0.0;
  direction.y() = 0.0;
  direction.z() = 1.0;

  robot_trajectory::RobotTrajectoryPtr result;

  std::cout << robot_state_ptr->getGlobalLinkTransform("right_hand").matrix()
  << std::endl;

  sawback_manipulation::solvers::CartesianPath cartesian_planner;
  bool valid = cartesian_planner.plan(robot_state_ptr, result, robot_model_ptr, joint_model_group_ptr, "right_hand",
                                      direction, true, 0.5);

  std::cout << robot_state_ptr->getGlobalLinkTransform("right_hand").matrix()
  << std::endl;

  moveit_cpp_ptr->execute(PLANNING_GROUP, result);

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
  return 0;
}

//
