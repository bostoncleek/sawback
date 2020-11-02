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
  ros::service::waitForService("/sawback_moveitcpp/get_grasp", -1);

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
  ROS_INFO_NAMED(LOGNAME, "New goal accepted: %s", goal_name_.c_str());

  // Either pick of place
  if (goal_name_ == "pick")
  {
    pickAction(goal->pre_distance, goal->post_distance);
  }

  else if (goal_name_ == "place")
  {
    placeAction(goal->place_pose, goal->pre_distance, goal->post_distance);
  }

  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Goal name not recognized");
    server_->setAborted();
  }
}

void SawbackPickPlace::preemptCallback()
{
  ROS_INFO_NAMED(LOGNAME, "Preempted %s:", goal_name_.c_str());
  server_->setPreempted();
}

void SawbackPickPlace::pickAction(double pre_distance, double post_distance)
{
  // Move to ready pose
  if (!readyPose())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to reach ready pose");
    server_->setAborted();
  }

  // Call grasping pipeline
  sawback_msgs::SampleGrasps grasp_srv;
  if (!grasp_client_.call(grasp_srv))
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to call get_grasp service");
    server_->setAborted();
  }

  const Eigen::Translation3d position(grasp_srv.response.grasp_candidate.pose.position.x + x_offset_,
                                      grasp_srv.response.grasp_candidate.pose.position.y + y_offset_,
                                      grasp_srv.response.grasp_candidate.pose.position.z + z_offset_);

  const Eigen::Quaterniond orientation(grasp_srv.response.grasp_candidate.pose.orientation.w,
                                       grasp_srv.response.grasp_candidate.pose.orientation.x,
                                       grasp_srv.response.grasp_candidate.pose.orientation.y,
                                       grasp_srv.response.grasp_candidate.pose.orientation.z);

  const Eigen::Isometry3d grasp = position * orientation;

  task_->initPick(pre_distance, post_distance, grasp);

  // Plan and execute task
  if (!task_->pick())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to pick object");
    server_->setAborted();
  }

  // Move to ready pose
  if (!readyPose())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to reach ready pose");
    server_->setAborted();
  }

  ROS_INFO_NAMED(LOGNAME, "Goal goal: %s success", goal_name_.c_str());
  sawback_msgs::PickPlaceResult result;
  result.status = true;
  server_->setSucceeded(result);
}

void SawbackPickPlace::placeAction(const geometry_msgs::PoseStamped& place_pose, double pre_distance,
                                   double post_distance)
{
  const Eigen::Translation3d position(place_pose.pose.position.x, place_pose.pose.position.y,
                                      place_pose.pose.position.z);

  const Eigen::Quaterniond orientation(place_pose.pose.orientation.w, place_pose.pose.orientation.x,
                                       place_pose.pose.orientation.y, place_pose.pose.orientation.z);

  const Eigen::Isometry3d place = position * orientation;

  task_->initPlace(pre_distance, post_distance, place);

  // Plan and execute task
  if (!task_->place())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to place object");
    server_->setAborted();
  }

  // Move to ready pose
  if (!readyPose())
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to reach ready pose");
    server_->setAborted();
  }

  ROS_INFO_NAMED(LOGNAME, "Goal goal: %s success", goal_name_.c_str());
  sawback_msgs::PickPlaceResult result;
  result.status = true;
  server_->setSucceeded(result);
}

bool SawbackPickPlace::readyPose()
{
  planning_component_ptr_->setStartStateToCurrentState();
  planning_component_ptr_->setGoal("ready");
  const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_ptr_->plan();

  if (plan.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    moveit_cpp_ptr_->execute(arm_planning_group_, plan.trajectory);
    return true;
  }

  return false;
}

}  // namespace sawback_manipulation
