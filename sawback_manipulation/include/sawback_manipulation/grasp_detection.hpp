/**
 * @file grasp_detection.hpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection using point clouds
 */

#pragma once

#include <string>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>

#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

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

  bool remove_ground_;     // specify if to remove table points
  bool cartesian_limits_;  // specify if to remove points outside limits

  std::string filtered_cloud_frame_;  // frame of filtered cloud

  Eigen::Matrix4f transform_base_optical_;

  Eigen::Isometry3d transfrom_base_camera_;     // transform from robot arm base to camera origin
  Eigen::Isometry3d transfrom_camera_optical_;  // transform from camera origin to camera optical link

  std::vector<double> xyz_lower_limits_;  // lower limits on point cloud
  std::vector<double> xyz_upper_limits_;  // upper limits on point cloud

  std::unique_ptr<gpd::GraspDetector> grasp_detector_;  // used to run the GPD algorithm
  std::unique_ptr<gpd::util::Cloud> cloud_camera_;      // stores point cloud with (optional) camera information
};

}  // namespace sawback_manipulation
