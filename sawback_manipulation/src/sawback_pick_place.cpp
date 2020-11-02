/**
 * @file sawback_pick_place.hpp
 * @author Boston Cleek
 * @date 13 Oct 2020
 * @brief Sawback pick and place pipline using moveitcpp
 */

#include <ros/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <sawback_msgs/SampleGrasps.h>
#include <sawback_pick_place.hpp>

namespace sawback_manipulation
{
constexpr char LOGNAME[] = "Sawback Pick Place";

SawbackPickPlace::SawbackPickPlace(const ros::NodeHandle& nh) : nh_(nh)
{
  loadParameters();
  init();
}

void SawbackPickPlace::loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading parameters");
  ros::NodeHandle pnh("~");
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_planning_group", arm_planning_group_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "gripper_planning_group", gripper_planning_group_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_link", eef_link_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "action_name", action_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "x_offset", x_offset_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "y_offset", y_offset_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "z_offset", z_offset_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void SawbackPickPlace::init()
{
  grasp_client_ = nh_.serviceClient<sawback_msgs::SampleGrasps>("get_grasp");
  // wait for service to become availible
  // ros::service::waitForService("/sawback_moveitcpp/get_grasp", -1);

  // MoveItCpp
  moveit_cpp_ptr_.reset(new moveit::planning_interface::MoveItCpp(nh_));
  moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

  planning_component_ptr_.reset(new moveit::planning_interface::PlanningComponent(arm_planning_group_, moveit_cpp_ptr_));

  // Pick and Place task
  task_.reset(new PickPlace(moveit_cpp_ptr_, arm_planning_group_, gripper_planning_group_, eef_link_));

  // action server
  server_.reset(new actionlib::SimpleActionServer<sawback_msgs::PickPlaceAction>(nh_, action_name_, false));
  server_->registerGoalCallback(std::bind(&SawbackPickPlace::goalCallback, this));
  server_->registerPreemptCallback(std::bind(&SawbackPickPlace::preemptCallback, this));
  server_->start();
}

void SawbackPickPlace::goalCallback()
{
  // Accept new goal
  sawback_msgs::PickPlaceGoalConstPtr goal = server_->acceptNewGoal();
  goal_name_ = goal->manipulation_task;
  geometry_msgs::PoseStamped pose = goal->pose;
  ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());

  // Either pick of place
}

void SawbackPickPlace::preemptCallback()
{
  ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
  server_->setPreempted();
}

}  // namespace sawback_manipulation
