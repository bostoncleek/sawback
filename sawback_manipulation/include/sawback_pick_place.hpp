/**
 * @file sawback_pick_place.hpp
 * @author Boston Cleek
 * @date 13 Oct 2020
 * @brief Sawback pick and place pipline using moveitcpp
 */

#pragma once

#include <memory>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <sawback_manipulation/pick_place.hpp>
#include <sawback_msgs/PickPlaceAction.h>

namespace sawback_manipulation
{
/** @brief Action server interface with pick and place pipeline */
class SawbackPickPlace
{
public:
  /**
  * @brief Constructor
  * @param nh - node handle
  */
  SawbackPickPlace(const ros::NodeHandle& nh);

private:
  /** @brief Load parameters */
  void loadParameters();

  /** @brief Initialize node */
  void init();

  /**
  * @brief Action server goal call back
  */
  void goalCallback();

  /**
  * @brief Preempts goal
  */
  void preemptCallback();

  /**
  * @brief Calls GPD and the pick pipeline
  * @param pre_distance - approach distance from grasp
  * @param post_distance - retreat distance from grasp
  */
  void pickAction(double pre_distance, double post_distance);

  /**
  * @brief Calls the place pipeline
  * @param place_pose - place pose
  * @param pre_distance - approach distance from grasp
  * @param post_distance - retreat distance from grasp
  */
  void placeAction(const geometry_msgs::PoseStamped& place_pose, double pre_distance, double post_distance);

  /**
  * @brief Moves arm to ready position
  * @return true if in ready position
  */
  bool readyPose();

private:
  ros::NodeHandle nh_;                                                       // moveitcpp node handle
  ros::ServiceClient grasp_client_;                                          // grasping client
  moveit::planning_interface::MoveItCppPtr moveit_cpp_ptr_;                  // moveitcpp pointer
  moveit::planning_interface::PlanningComponentPtr planning_component_ptr_;  // planning only for right arm

  std::unique_ptr<PickPlace> task_;                                                       // pick and place task
  std::unique_ptr<actionlib::SimpleActionServer<sawback_msgs::PickPlaceAction>> server_;  // pick and place action server

  std::string arm_planning_group_;      // planning group arm name
  std::string gripper_planning_group_;  // planning group gripper name
  std::string eef_link_;                // link name to plan the path for
  std::string action_name_;             // action name
  std::string goal_name_;               // goal name

  double x_offset_, y_offset_, z_offset_;  // offsets for grasping in base frame
};

}  // namespace sawback_manipulation
