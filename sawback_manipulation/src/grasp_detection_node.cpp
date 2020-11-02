/**
 * @file grasp_detection_node.cpp
 * @author Boston Cleek
 * @date 9 Oct 2020
 * @brief Grasp detection node using point clouds

  PARAMETERS:
    remove_ground - bool segment ground plane from point cloud
    cartesian_limits - bool removed point from cloud
    filtered_cloud_frame - frame ID of filtered point cloud
    path_to_gpd_config - path to GPD config file
    view_point - (x,y,z) view point of camera usually (0,0,0)
    transfrom_base_camera - transform from robot base to camera link
    transfrom_camera_optical - transform from camera link to optical link
    xyz_lower_limits (optional) - lower limits on point cloud if enabled cartesian_limits
    xyz_upper_limits (optional)-  upper limits on point cloud if enabled cartesian_limits

  PUBLISHES:
    filtered_cloud (sensor_msgs/PointCloud2) - filtered point cloud
    gpose (geometry_msgs/PoseStamped) - highest ranked grasp

  SUBSCRIBES:
    cloud (sensor_msgs/PointCloud2) - point cloud used for grasp detection

  SERVICES:
    get_grasp (sawback_msgs/SampleGrasps) - calls GPD alogrithm
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
