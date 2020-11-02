/**
 * @file grasp_detection_node.cpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection node using point clouds
 */

#include <grasp_detection.hpp>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("main", "Starting grasp_detection");
  ros::init(argc, argv, "grasp_detection");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  sawback_manipulation::GraspDetection grasp_detection(nh);
  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  return 0;
}
