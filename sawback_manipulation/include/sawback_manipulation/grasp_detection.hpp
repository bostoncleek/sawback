/**
 * @file grasp_detection.hpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection using point clouds
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace sawback_manipulation
{
class GraspDetection
{
public:
  GraspDetection(const ros::NodeHandle& nh);

private:
  void loadParameters();

  void init();

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;  // point cloud subscriber
  ros::Publisher cloud_pub_;   // publishes the point cloud saved

  std::vector<double> xyz_lower_limits_;  // lower limits on point cloud
  std::vector<double> xyz_upper_limits_;  // upper limits on point cloud
};

}  // namespace sawback_manipulation
