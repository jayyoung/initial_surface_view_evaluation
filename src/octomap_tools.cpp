#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "initial_surface_view_evaluation/ConvertCloudToOctree.h"

using namespace std;
using namespace octomap;
using namespace sensor_msgs;

octomap_msgs::Octomap convert_pcd_to_octomap(PointCloud2& input_cloud)
{
  ROS_INFO("Converting PointCloud to Octree");

  // when i was your age, we used strongly typed languages
  // "what's a type, grandad?"
  // well, let me show you
  float octree_resolution = 0.05f;
//  double octree_max_range = -1;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(input_cloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  ROS_INFO("- Computing centroid");
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*temp_cloud, centroid);
  octomap::point3d octo_centroid(centroid[0],centroid[1],centroid[2]);

  ROS_INFO("- Converting to OctoMap");
  // convert observation cloud to octomap and return it
  octomap::Pointcloud oct_pc;
  octomap::pointCloud2ToOctomap(input_cloud, oct_pc);
  octomap::OcTree map(octree_resolution);
  map.insertPointCloud(oct_pc, octo_centroid);

  ROS_INFO("- Writing to file");
  map.writeBinary("test.bt");

  ROS_INFO("- Converting to ROS msg");
  octomap_msgs::Octomap octo_msg;
  octomap_msgs::fullMapToMsg(map,octo_msg);
  ROS_INFO("- Done! Returning");
  return octo_msg;
}

bool convert_pcd_to_octomap_cb(
initial_surface_view_evaluation::ConvertCloudToOctree::Request &req, // phew! what a mouthful!
initial_surface_view_evaluation::ConvertCloudToOctree::Response &res)
{
  octomap_msgs::Octomap out = convert_pcd_to_octomap(req.cloud);
  res.octomap = out;
  return true;
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "surface_based_object_learnin_octomap_tools");
  ros::NodeHandle node;


  ROS_INFO("Setting up OcTree conversion service");
  ros::ServiceServer conversion_service = node.advertiseService("/surface_based_object_learning/convert_pcd_to_octomap", convert_pcd_to_octomap_cb);
  ROS_INFO("Done");



  ros::spin();
  return 0;
}
