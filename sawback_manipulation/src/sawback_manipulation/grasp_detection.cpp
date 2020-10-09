/**
 * @file grasp_detection.cpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection using point clouds
 */

#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <sawback_manipulation/grasp_detection.hpp>
#include <sawback_manipulation/perception/cloud_processing.hpp>

namespace sawback_manipulation
{
constexpr char LOGNAME[] = "Grasp Detection";

using perception::PointCloudRGB;
using perception::PointCloudRGBA;
using perception::removeGround;
using perception::passThroughFilter;

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
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1 /*, true*/);
}

void GraspDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // convert from ROS msg to a point cloud
  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  pcl::fromROSMsg(*msg.get(), *cloud.get());

  // segment object from ground
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
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud.get(), cloud_msg);
    cloud_pub_.publish(cloud_msg);
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Point cloud is empty");
  }
}

}  // namespace sawback_manipulation




















//