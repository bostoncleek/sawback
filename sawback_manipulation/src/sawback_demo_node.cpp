/**
* @file sawback_demo_node.cpp
* @author Boston Cleek
* @date 25 Sep 2020
* @brief Demo showing capabilities of sawback using moveitcpp
*/

// C++
#include <iostream>
#include <memory>

// ROS 
#include <ros/ros.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/PointStamped.h>


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
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  planning_components->setStartStateToCurrentState();

  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "base";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.28;
  target_pose1.pose.position.y = -0.2;
  target_pose1.pose.position.z = 0.5;

  planning_components->setGoal(target_pose1, "right_hand");

  auto plan_solution1 = planning_components->plan();

  if (plan_solution1)
  {
   ROS_INFO_STREAM_NAMED(LOGNAME, "Motion plan found");
   ROS_INFO_STREAM_NAMED(LOGNAME, "Press enter to continue");
   std::cin.get();
   planning_components->execute();
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
  return 0;
}
