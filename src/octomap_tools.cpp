#include <cstdlib>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/pfh.h"
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/registration/transforms.h>

#include "initial_surface_view_evaluation/CalculateOctreeOverlap.h"
#include "initial_surface_view_evaluation/ConvertCloudToOctomap.h"
#include "initial_surface_view_evaluation/ExtractNormalsFromOctomap.h"

#include "semantic_map_publisher/ObservationOctomapService.h"
#include "surface_based_object_learning/PointInROI.h"

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace octomap;
using namespace sensor_msgs;

#define ANGLE_MAX_DIFF (M_PI / 4)

octomap_msgs::Octomap convert_pcd_to_octomap(std::vector<sensor_msgs::PointCloud2> input_clouds)
{
  ROS_INFO("Converting PointCloud to Octree");
  ros::NodeHandle n;
  // when i was your age, we used strongly typed languages
  // "what's a type, grandad?"
  // well, let me show you
  float octree_resolution = 0.03f;
  octomap::OcTree map(octree_resolution);
  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("/initial_surface_view_evaluation/converted_octomaps", 5);

  geometry_msgs::Pose robot_pose = *ros::topic::waitForMessage<geometry_msgs::Pose>("/robot_pose", ros::Duration(5));

  // little bit of a hack, this is just a guesstimate of how high up the PTU is
  robot_pose.position.z = 1.76;


  for(std::vector<sensor_msgs::PointCloud2>::iterator iter = input_clouds.begin(), end = input_clouds.end(); iter != end; ++iter) {
      sensor_msgs::PointCloud2 input_cloud = *iter;

      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(input_cloud,pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

      if(temp_cloud->points.size() == 0) {
        ROS_INFO("Skipping empty point cloud");
        continue;
      } else {
          ROS_INFO("Processing cloud in set");
      }

//	ROS_INFO("Doing some noise reduction");
     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
     // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
      //outrem.setInputCloud(temp_cloud);
      //outrem.setRadiusSearch(0.8);
      //outrem.setMinNeighborsInRadius (5);
     // outrem.setNegative(false);
      //outrem.filter (*cloud_filtered_ptr);

/*
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> f;
      f.setInputCloud (temp_cloud);
      f.setMeanK (50);
      f.setStddevMulThresh (1.0);
      f.setNegative(false);
      f.filter(*cloud_filtered_ptr);
      */

     // pcl::PCDWriter writer;
    //  writer.write<pcl::PointXYZ> ("filtered.pcd", *cloud_filtered_ptr, false);
  //    temp_cloud = cloud_filtered_ptr;

      //ROS_INFO("- Computing centroid");
     // Eigen::Vector4f centroid;
    //  pcl::compute3DCentroid(*temp_cloud, centroid);

      // uh oh, should this be robot_pose?
    //  octomap::point3d octo_centroid(centroid[0],centroid[1],centroid[2]);
    octomap::point3d octo_centroid(robot_pose.position.x,robot_pose.position.y,robot_pose.position.z);

      //octomap::point3d octo_centroid(7.182,2.970,1.76);

      ROS_INFO("- Converting to OctoMap");
      // convert observation cloud to octomap and return it
      octomap::Pointcloud oct_pc;
      octomap::pointCloud2ToOctomap(input_cloud, oct_pc);

      map.insertPointCloud(oct_pc, octo_centroid);

  }


//  ROS_INFO("- Writing to file");
//  map.writeBinary("output.bt");
  ROS_INFO("- Converting to ROS msg");
  octomap_msgs::Octomap octo_msg;
  octo_msg.header.frame_id = "map";
  octomap_msgs::fullMapToMsg(map,octo_msg);
  octomap_pub.publish(octo_msg);
  ROS_INFO("- Done! Returning");
  return octo_msg;
}

// calculates the octomap overlap between source and target
// how many points from source are also in target?
float calculate_octo_overlap(sensor_msgs::PointCloud2 source, sensor_msgs::PointCloud2 target) {
  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);

  // Load the pair of point clouds
  //std::stringstream ss1, ss2;
  //ss1 << filename_base << "1.pcd";
  //pcl::io::loadPCDFile (ss1.str (), *points1);
  //ss2 << filename_base << "2.pcd";
  //pcl::io::loadPCDFile (ss2.str (), *points2);


  pcl::PCLPointCloud2 source_pc2;
  pcl_conversions::toPCL(source,source_pc2);
  pcl::fromPCLPointCloud2(source_pc2,*points1);

  pcl::PCLPointCloud2 target_pc2;
  pcl_conversions::toPCL(target,target_pc2);
  pcl::fromPCLPointCloud2(target_pc2,*points1);


  // Downsample the cloud
  const float voxel_grid_leaf_size = 0.01;
  downsample (points1, voxel_grid_leaf_size, downsampled1);
  downsample (points2, voxel_grid_leaf_size, downsampled2);
  // Compute surface normals
  const float normal_radius = 0.03;
  compute_surface_normals (downsampled1, normal_radius, normals1);
  compute_surface_normals (downsampled2, normal_radius, normals2);
  // Compute keypoints
  const float min_scale = 0.01;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  const float min_contrast = 10.0;
  detect_keypoints (points1, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints1);
  detect_keypoints (points2, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints2);
  // Compute PFH features
  const float feature_radius = 0.08;
  compute_PFH_features_at_keypoints (downsampled1, normals1, keypoints1, feature_radius, descriptors1);
  compute_PFH_features_at_keypoints (downsampled2, normals2, keypoints2, feature_radius, descriptors2);
  // Find feature correspondences
  std::vector<int> correspondences;
  std::vector<float> correspondence_scores;
  find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
  // Print out ( number of keypoints / number of points )
  std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
            << "out of " << downsampled1->size () << " total points." << std::endl;

  std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
            << "out of " << downsampled2->size () << " total points." << std::endl;
  // Visualize the two point clouds and their feature correspondences

  return correspondences.size();
}


void
downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}


void
compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);
  // Set the input points
  norm_est.setInputCloud (points);
  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}


void
detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                  float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
                  pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
  // Use a FLANN-based KdTree to perform neighborhood searches
  sift_detect.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Set the detection parameters
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  // Set the input
  sift_detect.setInputCloud (points);
  // Detect the keypoints and store them in "keypoints_out"
  sift_detect.compute (*keypoints_out);
}


void
compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;
  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (feature_radius);
  /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
   * use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
   * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
   * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
   * values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
   */

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);
  // Use all of the points for analyzing the local structure of the cloud
  pfh_est.setSearchSurface (points);
  pfh_est.setInputNormals (normals);
  // But only compute features at the keypoints
  pfh_est.setInputCloud (keypoints_xyzrgb);
  // Compute the features
  pfh_est.compute (*descriptors_out);
}


void
find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());
  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}



std::vector<geometry_msgs::Point> extract_normals_from_octomap(octomap_msgs::Octomap& in)
{
  ROS_INFO("Attempting to extract normals from octomap");
  std::vector<geometry_msgs::Point> output_points;
  AbstractOcTree* ab_tree = octomap_msgs::fullMsgToMap(in);

  if (!ab_tree){
    ROS_ERROR("Failed to recreate octomap");
    return output_points;
  }
  OcTree* tree = dynamic_cast<OcTree*>(ab_tree);
  OcTree* sp_tree = new OcTree(tree->getResolution());
  int free = 0;
  int occupied = 0;
  int supported = 0;

  ROS_INFO("Extracting supporting planes from octomap");

  for(OcTree::leaf_iterator it = tree->begin_leafs(),
        end=tree->end_leafs(); it!= end; ++it)
    {
      if (tree->isNodeOccupied(*it))
        {
          occupied++;
          std::vector<point3d> normals;

          point3d p3d = it.getCoordinate();

          bool got_normals = tree->getNormals(p3d ,normals, true);
          std::vector<point3d>::iterator normal_iter;

          point3d avg_normal (0.0, 0.0, 0.0);
          for(std::vector<point3d>::iterator normal_iter = normals.begin(),
                end = normals.end(); normal_iter!= end; ++normal_iter)
            {
              avg_normal+= (*normal_iter);
            }
          if (normals.size() > 0)
            {
              supported++;
              // cout << "#Normals: " << normals.size() << endl;
              avg_normal/= normals.size();
              point3d z_axis ( 0.0, 0.0, 1.0);
              double angle = avg_normal.angleTo(z_axis);
              point3d coord = it.getCoordinate();
	            double z = it.getZ();

              if ( angle < ANGLE_MAX_DIFF)
                {
                  sp_tree->updateNode(coord,true);
                  geometry_msgs::Point pt;
                  pt.x = coord.x();
                  pt.y = coord.y();
                  pt.z = coord.z();
                  output_points.push_back(pt);
                }
            }
        }
      else
        {
          free++;
        }
    }
  //sp_tree->writeBinary("post_alg.bt");
  ROS_INFO("Extracted map size: %i (%i free, and %i occupied leaf nodes were discarded)", supported, free, occupied - supported);
  return output_points;
}


bool extract_normals_from_octomap_cb(
initial_surface_view_evaluation::ExtractNormalsFromOctomap::Request &req, // phew! what a mouthful!
initial_surface_view_evaluation::ExtractNormalsFromOctomap::Response &res)
{
  std::vector<geometry_msgs::Point> out = extract_normals_from_octomap(req.octomap);
  res.up_facing_points = out;
  return true;
}


bool convert_pcd_to_octomap_cb(
initial_surface_view_evaluation::ConvertCloudToOctomap::Request &req, // phew! what a mouthful!
initial_surface_view_evaluation::ConvertCloudToOctomap::Response &res)
{
  octomap_msgs::Octomap out = convert_pcd_to_octomap(req.clouds);
  res.octomap = out;
  return true;
}

bool calculate_octree_overlap_cb(
initial_surface_view_evaluation::CalculateOctreeOverlap::Request &req, // phew! what a mouthful!
initial_surface_view_evaluation::CalculateOctreeOverlap::Response &res)
{
  float out = calculate_octo_overlap(req.source,req.target);
  res.degree_of_overlap = out;
  return true;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "surface_based_object_learnin_octomap_tools");
  ros::NodeHandle node;

  ROS_INFO("Setting up OcTree conversion services");
  ros::ServiceServer conversion_service = node.advertiseService("/surface_based_object_learning/convert_pcd_to_octomap", convert_pcd_to_octomap_cb);
  ros::ServiceServer extract_service = node.advertiseService("/surface_based_object_learning/extract_normals_from_octomap", extract_normals_from_octomap_cb);
  ros::ServiceServer octree_overlap = node.advertiseService("/surface_based_object_learning/calculate_octree_overlap", calculate_octree_overlap_cb);
  ROS_INFO("Done");



  ros::spin();
  return 0;
}
