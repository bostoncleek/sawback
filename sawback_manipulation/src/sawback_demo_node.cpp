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

// ROS
#include <ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/cartesian_interpolator.h>

#include <moveit/robot_trajectory/robot_trajectory.h>


namespace rvt = rviz_visual_tools;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sawback_moveitcpp");
  ros::NodeHandle nh("/sawback_moveitcpp");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  static const std::string PLANNING_GROUP = "right_arm";
  static const std::string LOGNAME = "sawback_moveitcpp";

  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting...");

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components =
      std::make_shared<moveit::planning_interface::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_state_ptr = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);


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

  // ROS_INFO_STREAM_NAMED("Root link: %s", robot_model_ptr->getRootLinkName().c_str());
  // ROS_INFO_STREAM_NAMED("Model frame: %s", robot_model_ptr->getModelFrame ().c_str());
  //
  // std::cout << robot_state_ptr->getGlobalLinkTransform("right_hand").matrix() << std::endl;
  //
  // bool frame_found = false;
  // bool *frame_found_ptr = &frame_found;
  // std::cout << robot_state_ptr->getFrameTransform("right_hand", frame_found_ptr).matrix() << std::endl;
  // std::cout << "Frame found: " << frame_found << std::endl;

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

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to perfrom cartesian planning");


  planning_components->setStartStateToCurrentState();
  robot_state_ptr = moveit_cpp_ptr->getCurrentState();

  Eigen::Vector3d direction;
  direction.x() = 0.0;
  direction.y() = 0.0;
  direction.z() = 1.0;

  moveit::core::MaxEEFStep max_step(0.01);
  moveit::core::JumpThreshold jmp_thresh(1.5);


  const moveit::core::LinkModel* link_model_ptr = joint_model_group_ptr->getLinkModel("right_hand");
  std::vector<moveit::core::RobotStatePtr> trajectory;

  const double frac = moveit::core::CartesianInterpolator::computeCartesianPath(robot_state_ptr.get(),
                                                                                joint_model_group_ptr,
                                                                                trajectory,
                                                                                link_model_ptr,
                                                                                direction,
                                                                                true,
                                                                                0.2,
                                                                                max_step,
                                                                                jmp_thresh);

  ROS_INFO_NAMED(LOGNAME, "Fraction of trajectory %f: ", frac);
  ROS_INFO_NAMED(LOGNAME, "length of trajectory %ld: ", trajectory.size());

  robot_trajectory::RobotTrajectoryPtr result;
  result.reset(new robot_trajectory::RobotTrajectory(moveit_cpp_ptr->getRobotModel(), joint_model_group_ptr));

  for (const auto& waypoint : trajectory)
  {
    result->addSuffixWayPoint(waypoint, 0.0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization timing;
  timing.computeTimeStamps(*result, 1.0, 1.0);

  moveit_cpp_ptr->execute(PLANNING_GROUP, result);

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
  return 0;
}












//
