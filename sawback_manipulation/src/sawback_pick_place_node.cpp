/**
 * @file sawback_pick_place_node.cpp
 * @author Boston Cleek
 * @date 25 Sep 2020
 * @brief Sawback pick and place pipline using moveitcpp

   PARAMETERS:
     arm_planning_group - arm planning group name
     gripper_planning_group - gripper planning group name
     eef_link - end effector name
     action_name - action name
     x_offset - grasp x-axis offset
     y_offset - grasp y-axis offset
     z_offset - grasp z-axis offset

   SERVICES:
     get_grasp (sawback_msgs/PickPlace) - pick and place action 
 */

#include <sawback_pick_place.hpp>
#include <sawback_msgs/SampleGrasps.h>

#include <eigen_conversions/eigen_msg.h>

// moveit_msgs::CollisionObject createObject()
// {
//   moveit_msgs::CollisionObject object;
//   object.id = "box";
//   object.header.frame_id = "base";
//   object.primitives.resize(1);
//   object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//   object.primitives[0].dimensions.resize(3);
//   object.primitives[0].dimensions[0] = 0.2;
//   object.primitives[0].dimensions[1] = 0.2;
//   object.primitives[0].dimensions[2] = 0.2;
//   object.primitive_poses.resize(1);
//   object.primitive_poses[0].position.x = 0.7;
//   object.primitive_poses[0].position.y = 0.0;
//   object.primitive_poses[0].position.z = -0.2;
//   object.primitive_poses[0].orientation.w = 1.0;
//   object.operation = moveit_msgs::CollisionObject::ADD;
//
//   return object;
// }

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("main", "Starting sawback_pick_place");
  ros::init(argc, argv, "sawback_pick_place");
  ros::NodeHandle nh("/sawback_moveitcpp");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Otherwise robot with zeros joint_states */
  ros::Duration(1.0).sleep();

  sawback_manipulation::SawbackPickPlace sawback_pick_place(nh);

  // ros::NodeHandle pnh("~");
  // double xoffset = 0.0;
  // double yoffset = 0.0;
  // double zoffset = 0.0;
  //
  // rosparam_shortcuts::get(LOGNAME, pnh, "xoffset", xoffset);
  // rosparam_shortcuts::get(LOGNAME, pnh, "yoffset", yoffset);
  // rosparam_shortcuts::get(LOGNAME, pnh, "zoffset", zoffset);

  // ros::ServiceClient grasp_client = nh.serviceClient<sawback_msgs::SampleGrasps>("get_grasp");
  // ros::service::waitForService("/sawback_moveitcpp/get_grasp", -1);
  //
  // auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  // moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
  //
  // moveit::planning_interface::PlanningComponentPtr planning_component_ptr =
  //     std::make_shared<moveit::planning_interface::PlanningComponent>("right_arm", moveit_cpp_ptr);
  //
  // planning_component_ptr->setStartStateToCurrentState();
  // // planning_component_ptr->setGoal(const_cast<const robot_state::RobotState&>(*robot_state_ptr.get()));
  // planning_component_ptr->setGoal("ready");
  //
  // const moveit::planning_interface::PlanningComponent::PlanSolution plan1 = planning_component_ptr->plan();
  //
  // if (plan1.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  // {
  //   moveit_cpp_ptr->execute("right_arm", plan1.trajectory);
  // }
  //
  // else
  // {
  //   ros::shutdown();
  // }
  //
  // // Begin manipulation tasks
  // auto task =
  //     std::make_unique<sawback_manipulation::PickPlace>(moveit_cpp_ptr, "right_arm", "hand", "right_gripper_tip");
  //
  // // Call grasping pipeline
  // sawback_msgs::SampleGrasps grasp_srv;
  //
  // if (!grasp_client.call(grasp_srv))
  // {
  //   ROS_ERROR("Failed to call get_grasp service");
  //   ros::shutdown();
  // }
  //
  // const Eigen::Translation3d position(grasp_srv.response.grasp_candidate.pose.position.x,
  //                                     grasp_srv.response.grasp_candidate.pose.position.y,
  //                                     grasp_srv.response.grasp_candidate.pose.position.z);
  //
  // const Eigen::Quaterniond orientation(grasp_srv.response.grasp_candidate.pose.orientation.w,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.x,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.y,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.z);
  //
  // // const Eigen::Translation3d position(0.907281 + xoffset, -0.0134618 + yoffset, -0.193109 + zoffset);
  // // const Eigen::Translation3d position(0.907281, -0.0134618, -0.193109 );
  // // const Eigen::Quaterniond orientation(-0.16374, -0.638497, -0.185033, 0.728885);
  //
  // const Eigen::Isometry3d grasp = position * orientation;
  //
  // // const double roll = 0.0, pitch = 1.5708 , yaw = 0.0;//1.5708 * 2.0;
  // // const Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
  // //                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
  // //                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  // //
  // // // const Eigen::Quaterniond quat(1.0, 0.0, 0.0, 0.0);
  // // Eigen::Isometry3d grasp = Eigen::Translation3d(0.8, 0.1, -0.15) * quat;
  //
  // task->initPick(0.15, 0.15, grasp);
  // task->pick();
  //
  // task->initPlace(0.15, 0.15, grasp);
  // task->place();

  // const Eigen::Quaterniond quat(-0.0336479, 0.677444, -0.0578169, 0.732526);
  // const Eigen::Isometry3d grasp = Eigen::Translation3d(0.747486, 0.101057, 0.724942) * quat;

  // moveit::core::RobotStatePtr robot_start_state_ptr = moveit_cpp_ptr->getCurrentState();
  //
  // const Eigen::Isometry3d Troot_base = robot_start_state_ptr->getGlobalLinkTransform("base");
  //
  // grasp = Troot_base * grasp;
  //
  //
  // const Eigen::Isometry3d start_pose = robot_start_state_ptr->getGlobalLinkTransform("right_hand");
  // const Eigen::Quaterniond rot(start_pose.rotation());
  //
  // // std::cout << "Position" << std::endl;
  // // std::cout << start_pose.translation() << std::endl;
  // // std::cout << "Orientation" << std::endl;
  // //
  // // std::cout << rot.w() << std::endl;
  // // std::cout << rot.x() << std::endl;
  // // std::cout << rot.y() << std::endl;
  // // std::cout << rot.z() << std::endl;
  //
  // const moveit::core::RobotStateConstPtr robot_start_state =
  //     std::const_pointer_cast<const moveit::core::RobotState>(robot_start_state_ptr);
  //
  // geometry_msgs::PoseStamped target;
  // target.header.frame_id = "base_link";
  //
  // tf::poseEigenToMsg(grasp, target.pose);
  //
  // // Plan from start state
  // planning_component_ptr->setStartState(*robot_start_state.get());
  // planning_component_ptr->setGoal(target, "right_hand");
  // const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_ptr->plan();
  //
  // moveit_cpp_ptr->execute("right_arm", plan.trajectory);

  ros::waitForShutdown();
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");

  return 0;
}

//
