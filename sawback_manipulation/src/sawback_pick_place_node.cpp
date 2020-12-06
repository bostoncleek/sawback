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

  ros::waitForShutdown();
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");

  return 0;
}

//
