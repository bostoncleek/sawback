/**
 * @file grasp_detection.cpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection using point clouds
 */

#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <sawback_manipulation/grasp_detection.hpp>
#include <sawback_manipulation/perception/cloud_processing.hpp>

namespace sawback_manipulation
{
constexpr char LOGNAME[] = "Grasp Detection";

using perception::passThroughFilter;
using perception::PointCloudRGB;
using perception::PointCloudRGBA;
using perception::removeGround;

GraspDetection::GraspDetection(const ros::NodeHandle& nh) : nh_(nh)
{
  loadParameters();
  init();
}

void GraspDetection::loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading grasp detection parameters");
  ros::NodeHandle pnh("~");
  size_t errors = 0;

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "remove_ground", remove_ground_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "cartesian_limits", cartesian_limits_);

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "filtered_cloud_frame", filtered_cloud_frame_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "path_to_gpd_config", path_to_gpd_config_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "view_point", view_point_);

  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "transfrom_base_camera", transfrom_base_camera_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "transfrom_camera_optical", transfrom_camera_optical_);

  if (cartesian_limits_)
  {
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "xyz_lower_limits", xyz_lower_limits_);
    errors += !rosparam_shortcuts::get(LOGNAME, pnh, "xyz_upper_limits", xyz_upper_limits_);
  }
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void GraspDetection::init()
{
  cloud_sub_ = nh_.subscribe("cloud", 1, &GraspDetection::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1, true);
  grasp_srv_ = nh_.advertiseService("get_grasp", &GraspDetection::graspCallback, this);

  grasp_detector_.reset(new gpd::GraspDetector(path_to_gpd_config_));

  Eigen::Isometry3d Tbo = transfrom_base_camera_ * transfrom_camera_optical_;

  transform_base_optical_ = Eigen::Matrix4f::Identity();
  transform_base_optical_.block<3, 3>(0, 0) = Tbo.rotation().cast<float>();
  transform_base_optical_.block<3, 1>(0, 3) = Tbo.translation().cast<float>();
}

void GraspDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  cloud_msg_ = msg;
}

bool GraspDetection::graspCallback(sawback_msgs::SampleGrasps::Request&, sawback_msgs::SampleGrasps::Response& res)
{
  ROS_INFO_NAMED(LOGNAME, "Grasp service active");

  PointCloudRGBA::Ptr grasp_cloud(new PointCloudRGBA);
  processCloud(grasp_cloud);
  sampleGrasps(grasp_cloud);

  return true;
}

void GraspDetection::processCloud(PointCloudRGBA::Ptr& grasp_cloud)
{
  ROS_INFO_NAMED(LOGNAME, "cloud");

  // Transform incoming cloud into base frame
  sensor_msgs::PointCloud2 transformed_cloud;
  pcl_ros::transformPointCloud(transform_base_optical_, *cloud_msg_.get(), transformed_cloud);

  transformed_cloud.header.frame_id = filtered_cloud_frame_;

  // convert from ROS msg to a point cloud
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  pcl::fromROSMsg(transformed_cloud, *cloud.get());

  // segment object from ground
  // Segementation works best with XYZRGB
  if (remove_ground_)
  {
    removeGround(cloud);
  }

  // remove points out of limits
  if (cartesian_limits_)
  {
    passThroughFilter(xyz_lower_limits_, xyz_upper_limits_, cloud);
  }

  // publish the cloud for visualization and debugging purposes
  if (!cloud->points.empty())
  {
    pcl::toROSMsg(*cloud.get(), transformed_cloud);
    cloud_pub_.publish(transformed_cloud);

    // copy
    pcl::copyPointCloud(*cloud.get(), *grasp_cloud.get());
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Point cloud is empty");
  }
}


void GraspDetection::sampleGrasps(const PointCloudRGBA::Ptr& grasp_cloud)
{
  // Construct the cloud camera
  Eigen::Matrix3Xd camera_view_point(3, 1);
  camera_view_point << view_point_.at(0), view_point_.at(1), view_point_.at(2);
  cloud_camera_.reset(new gpd::util::Cloud(grasp_cloud, 0, camera_view_point));

  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps;  // detect grasp poses
  grasp_detector_->preprocessPointCloud(*cloud_camera_.get());      // preprocess the point cloud
  grasps = grasp_detector_->detectGrasps(*cloud_camera_.get());     // detect grasps in the point cloud

  for(const auto& grasp : grasps)
  {
    std::cout << "----" << std::endl;
    std::cout << grasp->getOrientation().col(2) << std::endl;
  }

}

















}  // namespace sawback_manipulation

//
