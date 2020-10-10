/**
 * @file cloud_processing.cpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Basic functionality for processing point clouds
 */

#include <ros/console.h>

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <sawback_manipulation/perception/cloud_processing.hpp>

namespace sawback_manipulation
{
namespace perception
{
constexpr char LOGNAME[] = "Cloud Processing";

void removeGround(PointCloudRGB::Ptr& cloud)
{
  // SAC segmentor without normals
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);

  // Max iterations and model tolerance
  segmentor.setMaxIterations(1000);
  segmentor.setDistanceThreshold(0.01);

  // Input cloud
  segmentor.setInputCloud(cloud);

  // Inliers representing points in the plane
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Use a plane as the model for the segmentor
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane.get(), *coefficients_plane.get());

  if (inliers_plane->indices.size() == 0)
  {
    ROS_ERROR_NAMED(LOGNAME, "Could not estimate a planar model for the given dataset");
  }

  // Extract the inliers from the cloud
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);

  // Remove plane inliers and extract the rest
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud.get());

  if (cloud->points.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Point cloud is empty, segementation failed");
  }
}

void passThroughFilter(const std::vector<double>& xyz_lower, const std::vector<double>& xyz_upper,
                       PointCloudRGB::Ptr& cloud)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(xyz_lower.at(0), xyz_upper.at(0));
  pass.filter(*cloud.get());

  pass.setFilterFieldName("y");
  pass.setFilterLimits(xyz_lower.at(1), xyz_upper.at(1));
  pass.filter(*cloud.get());

  pass.setFilterFieldName("z");
  pass.setFilterLimits(xyz_lower.at(2), xyz_upper.at(2));
  pass.filter(*cloud.get());
}

}  // namespace perception
}  // namespace sawback_manipulation
