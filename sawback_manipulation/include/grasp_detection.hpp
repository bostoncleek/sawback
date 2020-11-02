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
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Geometry>

#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

#include <sawback_msgs/SampleGrasps.h>

namespace sawback_manipulation
{
/** @brief Grasp detection pipeline */
class GraspDetection
{
public:
  /**
   * @brief Constructor
   * @param nh - node handle
   */
  GraspDetection(const ros::NodeHandle& nh);

private:
  /** @brief Load parameters */
  void loadParameters();

  /** @brief Initialize node */
  void init();

  /**
   * @brief Stereo camera point cloud call back
   * @param msg - point cloud message
   */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  /**
   * @brief Detect grasps service call back
   * @param res[out] - result containing the highest ranked grasp
   * @return true if successfully detected grasps
   */
  bool graspCallback(sawback_msgs::SampleGrasps::Request&, sawback_msgs::SampleGrasps::Response& res);

  /**
   * @brief Perform point cloud pre-processing before GPD
   * @param grasp_cloud[out] - processed point cloud
   * @details Can segment ground plan and restrict point cloud based on cartesian limits
   */
  void processCloud(PointCloudRGBA::Ptr& grasp_cloud);

  /**
   * @brief Calls GPD grasp pose detection algorithm
   * @param grasp[out] - highest ranked grasp
   * @param grasp_cloud - point cloud used to detect grasps
   * @return true if grasps are detected
   */
  bool sampleGrasps(geometry_msgs::PoseStamped& grasp, const PointCloudRGBA::Ptr& grasp_cloud);

private:
  ros::NodeHandle nh_;               // node handle
  ros::Subscriber cloud_sub_;        // point cloud subscriber
  ros::Publisher cloud_pub_;         // publishes the point cloud saved
  ros::ServiceServer grasp_server_;  // grasp server

  ros::Publisher pose_pub_;  // pub;ish grasp pose

  bool remove_ground_;     // specify if to remove table points
  bool cartesian_limits_;  // specify if to remove points outside limits

  std::string filtered_cloud_frame_;  // frame of filtered cloud
  std::string path_to_gpd_config_;    // path to GPD config file

  std::vector<double> view_point_;  // origin of the camera

  Eigen::Matrix4f transform_base_optical_;  // transform from base to left optical link

  Eigen::Isometry3d transfrom_base_camera_;     // transform from robot arm base to camera origin
  Eigen::Isometry3d transfrom_camera_optical_;  // transform from camera origin to camera optical link

  std::vector<double> xyz_lower_limits_;  // lower limits on point cloud
  std::vector<double> xyz_upper_limits_;  // upper limits on point cloud

  std::unique_ptr<gpd::GraspDetector> grasp_detector_;  // used to run the GPD algorithm
  std::unique_ptr<gpd::util::Cloud> cloud_camera_;      // stores point cloud with (optional) camera information

  sensor_msgs::PointCloud2::ConstPtr cloud_msg_;  // store the recent point cloud
};

}  // namespace sawback_manipulation
