/**
 * @file sawback_pick_place_node.cpp
 * @author Boston Cleek
 * @date 25 Sep 2020
 * @brief Sawback pick and place pipline using moveitcpp
 */

#include <sawback_manipulation/sawback_pick_place.hpp>

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

  // sawback_manipulation::SawbackPickPlace sawback_pick_place(nh);

  // ros::NodeHandle pnh("~");
  // double xoffset = 0.0;
  // double yoffset = 0.0;
  // double zoffset = 0.0;
  //
  // rosparam_shortcuts::get(LOGNAME, pnh, "xoffset", xoffset);
  // rosparam_shortcuts::get(LOGNAME, pnh, "yoffset", yoffset);
  // rosparam_shortcuts::get(LOGNAME, pnh, "zoffset", zoffset);
  //
  /* Otherwise robot with zeros joint_states */
  // ros::Duration(1.0).sleep();


  // ros::ServiceClient grasp_client = nh.serviceClient<sawback_msgs::SampleGrasps>("get_grasp");
  // ros::service::waitForService("/sawback_moveitcpp/get_grasp", -1);

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  moveit::planning_interface::PlanningComponentPtr planning_component_ptr =
      std::make_shared<moveit::planning_interface::PlanningComponent>("right_arm", moveit_cpp_ptr);

  // TODO: add floor and object
  // const moveit_msgs::CollisionObject box = createObject();

  // const double initial_joints[] = { 0.136716796875, -1.345919921875,  -0.3328642578125, 2.0867880859375,
  //                                   -0.19401171875, -0.8204111328125, -1.0447021484375 };

  // move to starting configuration
  // const moveit::core::RobotModelConstPtr robot_model =
  //     std::const_pointer_cast<const moveit::core::RobotModel>(moveit_cpp_ptr->getRobotModel());
  //
  // moveit::core::RobotStatePtr robot_state_ptr = make_shared<moveit::core::RobotState>(robot_model);
  //
  // // Set joint angles to positions from IK
  // robot_state_ptr->setJointGroupPositions("right_arm", initial_joints);
  // // Update all the transforms
  // robot_state_ptr->update();

  planning_component_ptr->setStartStateToCurrentState();
  // planning_component_ptr->setGoal(const_cast<const robot_state::RobotState&>(*robot_state_ptr.get()));
  planning_component_ptr->setGoal("ready");

  const moveit::planning_interface::PlanningComponent::PlanSolution plan = planning_component_ptr->plan();

  if (plan.error_code == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    moveit_cpp_ptr->execute("right_arm", plan.trajectory);
  }

  else
  {
    ros::shutdown();
  }

  // Begin manipulation tasks
  // auto task = std::make_unique<sawback_manipulation::tasks::PickPlace>(moveit_cpp_ptr, "right_arm", "hand",
  //                                                                      "right_gripper_tip");
  //
  // // Call grasping pipeline
  // sawback_msgs::SampleGrasps grasp_srv;
  //
  // if (!grasp_client.call(grasp_srv))
  // {
  //   ROS_ERROR_NAMED(LOGNAME, "Failed to call get_grasp service");
  //   ros::shutdown();
  // }
  //
  // const Eigen::Translation3d position(grasp_srv.response.grasp_candidate.pose.position.x + xoffset,
  //                                     grasp_srv.response.grasp_candidate.pose.position.y + yoffset,
  //                                     grasp_srv.response.grasp_candidate.pose.position.z + zoffset);
  //
  // const Eigen::Quaterniond orientation(grasp_srv.response.grasp_candidate.pose.orientation.w,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.x,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.y,
  //                                      grasp_srv.response.grasp_candidate.pose.orientation.z);
  //
  // // const Eigen::Translation3d position(0.907281 + xoffset, -0.0134618 + yoffset, -0.193109 + zoffset);
  // // const Eigen::Quaterniond orientation(-0.16374, -0.638497, -0.185033, 0.728885);
  //
  // const Eigen::Isometry3d grasp = position * orientation;
  //
  // // const double roll = 0.0, pitch = 1.5708 * 2.0, yaw = 1.5708 * 2.0;
  // // const Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
  // //                                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
  // //                                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  // //
  // // const Eigen::Isometry3d grasp = Eigen::Translation3d(0.8, 0.1, -0.25) * quat;
  //
  // task->initPick(0.15, 0.15, grasp);
  // task->planPick();

  // task->initPlace(0.15, 0.15, grasp);
  // task->planPlace();

  // task->execute();

  ros::waitForShutdown();
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");

  return 0;
}

//
