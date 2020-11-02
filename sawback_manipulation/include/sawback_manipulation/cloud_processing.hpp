/**
 * @file cloud_processing.hpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Basic functionality for processing point clouds
 */

#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sawback_manipulation
{
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

/**
 * @brief Segments objects from table plane
 * @param [out] cloud - Segemented point cloud XYZRGB
 */
void removeGround(PointCloudRGB::Ptr& cloud);

/**
 * @brief Removes points outside the specified cartesian limits
 * @param xyz_lower - 3dim vector x,y,z lower limits on cloud
 * @param xyz_upper - 3dim vector x,y,z upper limits on cloud
 * @param [out] cloud - cloud XYZRGB
 */
void passThroughFilter(const std::vector<double>& xyz_lower, const std::vector<double>& xyz_upper,
                       PointCloudRGB::Ptr& cloud);

}  // namespace sawback_manipulation
