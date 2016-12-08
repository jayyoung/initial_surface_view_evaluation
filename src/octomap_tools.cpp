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
#include <pcl/features/fpfh.h>
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <iostream>
#include <fstream>
#include "initial_surface_view_evaluation/CalculateOctreeOverlap.h"
#include "initial_surface_view_evaluation/ConvertCloudToOctomap.h"
#include "initial_surface_view_evaluation/ExtractNormalsFromOctomap.h"

#include "semantic_map_publisher/ObservationOctomapService.h"
#include "surface_based_object_learning/PointInROI.h"

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "pcl/common/geometry.h"
#include "pcl/common/distances.h"

using namespace std;
using namespace octomap;
using namespace sensor_msgs;

#define ANGLE_maximum_DIFF (M_PI / 4)



octomap_msgs::Octomap convert_pcd_to_octomap(std::vector<sensor_msgs::PointCloud2> input_clouds, float resolution)
{
  ROS_INFO("Converting PointCloud to Octree");
  ros::NodeHandle n;
  octomap::Pointcloud oct_pc;
    octomap_msgs::Octomap octo_msg;
  // when i was your age, we used strongly typed languages
  // "what's a type, grandad?"
  // well, let me show you
  float octree_resolution = resolution;
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
        if(input_clouds.size() == 1) {
          ROS_INFO("Only had one point cloud, and it was empty. Short-circuiting view planning.");
          return octo_msg;
        }
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

      octomap::pointCloud2ToOctomap(input_cloud, oct_pc);

      map.insertPointCloud(oct_pc, octo_centroid);

  }


//  ROS_INFO("- Writing to file");
//  map.writeBinary("output.bt");
  ROS_INFO("- Converting to ROS msg");

  octo_msg.header.frame_id = "map";
  octomap_msgs::fullMapToMsg(map,octo_msg);
  octomap_pub.publish(octo_msg);
  ROS_INFO("- Done! Returning");
  return octo_msg;
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

              if ( angle < ANGLE_maximum_DIFF)
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






void downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
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
  std::cout << "computing normals of cloud of size " << points->size() << std::endl;
  norm_est.setInputCloud (points);
  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}


void
detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                  float minimum_scale, int nr_octaves, int nr_scales_per_octave, float minimum_contrast,
                  pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;
  // Use a FLANN-based KdTree to perform neighborhood searches
  sift_detect.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Set the detection parameters
  sift_detect.setScales (minimum_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (minimum_contrast);
  // Set the input
  sift_detect.setInputCloud (points);
  // Detect the keypoints and store them in "keypoints_out"
  sift_detect.compute (*keypoints_out);
}


void
compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                                   pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> pfh_est;
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
find_feature_correspondences (pcl::PointCloud<pcl::SHOT352>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::SHOT352>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());
  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::SHOT352> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    // k > 0.75 trick?
    if (!pcl_isfinite (source_descriptors->at(i).descriptor[0])) //skipping NaNs
     {
       continue;
     }
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}



float add_calculate_octo_overlap(sensor_msgs::PointCloud2 source, sensor_msgs::PointCloud2 target) {


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  ROS_INFO("Set up temporary clouds");
  // Load the pair of point clouds
  //std::stringstream ss1, ss2;
  //ss1 << filename_base << "1.pcd";
  //pcl::io::loadPCDFile (ss1.str (), *points1);
  //ss2 << filename_base << "2.pcd";
  //pcl::io::loadPCDFile (ss2.str (), *points2);

  ROS_INFO("Converting to correct format");
  pcl::PCLPointCloud2 source_pc2;
  pcl_conversions::toPCL(source,source_pc2);
  pcl::fromPCLPointCloud2(source_pc2,*points1);

  pcl::PCLPointCloud2 target_pc2;
  pcl_conversions::toPCL(target,target_pc2);
  pcl::fromPCLPointCloud2(target_pc2,*points2);
  ROS_INFO("Done!");

  const float voxel_grid_leaf_size = 0.003;
  downsample(points1, voxel_grid_leaf_size, downsampled1);

  ROS_INFO("Downsampling input B");
  downsample(points2, voxel_grid_leaf_size, downsampled2);

  int overlapping_nodes = 0;
  int source_nodes = 0;

  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = downsampled1->begin(); it  != downsampled1->end(); it++){
    pcl::PointXYZRGB source_point = *it;
    source_nodes++;
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator itt = downsampled2->begin(); itt  != downsampled2->end(); itt++){
      pcl::PointXYZRGB target_point = *itt;
      float dist = pcl::geometry::distance(source_point.getArray3fMap(),target_point.getArray3fMap());
      if(dist <= voxel_grid_leaf_size) {
        overlapping_nodes++;
        break;
      }
    }
  }




  cout << " -- " << endl;
  cout << "points in source cloud: " << source_nodes << endl;
  cout << "overlapping nodes: " << overlapping_nodes << endl;


  // if one is smaller than the other, then the very best we can do is to match
  // all of its points
  float smallest = downsampled1->size();
  if(downsampled1->size() > downsampled2->size()) {
    smallest = downsampled2->size();
  }

  cout << "max possible overlapping nodes: " << smallest << endl;


  float degree = (1.0/smallest)*overlapping_nodes;

  cout << "degree of overlap: " << degree << endl;
  cout << " -- " << endl;


  return degree;
}

// calculates the octomap overlap between source and target
// how many points from source are also in target?
float calculate_octo_overlap(sensor_msgs::PointCloud2 source, sensor_msgs::PointCloud2 target) {
  // Create some new point clouds to hold our data
  ROS_INFO("Attempting to calculate likeness score");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors1 (new pcl::PointCloud<pcl::SHOT352>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors2 (new pcl::PointCloud<pcl::SHOT352>);
  ROS_INFO("Set up temporary clouds");
  // Load the pair of point clouds
  //std::stringstream ss1, ss2;
  //ss1 << filename_base << "1.pcd";
  //pcl::io::loadPCDFile (ss1.str (), *points1);
  //ss2 << filename_base << "2.pcd";
  //pcl::io::loadPCDFile (ss2.str (), *points2);

  ROS_INFO("Converting to correct format");
  pcl::PCLPointCloud2 source_pc2;
  pcl_conversions::toPCL(source,source_pc2);
  pcl::fromPCLPointCloud2(source_pc2,*points1);

  pcl::PCLPointCloud2 target_pc2;
  pcl_conversions::toPCL(target,target_pc2);
  pcl::fromPCLPointCloud2(target_pc2,*points2);
  ROS_INFO("Done!");


  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (points1);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*points1_filtered);

  sor.setInputCloud (points2);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*points2_filtered);



  std::cout << "source has " << points1->size() << " points before filtering" << std::endl;
  std::cout << "target has " << points2->size() << " points before filtering" << std::endl;

  std::cout << "source has " << points1_filtered->size() << " points after filtering" << std::endl;
  std::cout << "target has " << points2_filtered->size() << " points after filtering" << std::endl;


  ROS_INFO("Downsampling input A");
  // Downsample the cloud
  const float voxel_grid_leaf_size = 0.01;
  downsample(points1_filtered, voxel_grid_leaf_size, downsampled1);
  //downsample(1.0);
  ROS_INFO("Downsampling input B");
  downsample(points2_filtered, voxel_grid_leaf_size, downsampled2);
  // Compute surface normals

  ROS_INFO("Done!");

  ROS_INFO("Computing surface normals");
  const float normal_radius = 0.02;
  compute_surface_normals (downsampled1, normal_radius, normals1);
  ROS_INFO("Done input A");
  compute_surface_normals (downsampled2, normal_radius, normals2);
    ROS_INFO("Done input B");
  // Compute keypoints
  const float minimum_scale = 0.01;
  const int nr_octaves = 5;
  const int nr_octaves_per_scale = 5;
  const float minimum_contrast = 3.0;
  ROS_INFO("Detecting keypoints");
  detect_keypoints (downsampled1, minimum_scale, nr_octaves, nr_octaves_per_scale, minimum_contrast, keypoints1);
  detect_keypoints (downsampled2, minimum_scale, nr_octaves, nr_octaves_per_scale, minimum_contrast, keypoints2);
  ROS_INFO("Done! Woo we're getting close");

  bool skip_visuals = false;
    if(keypoints1->size() == 0 || keypoints2->size() == 0) {
      ROS_INFO("No keypoints found, nothing we can do about this.");
      skip_visuals = true;
    }

    float octo_score = add_calculate_octo_overlap(source,target);

if(skip_visuals == false) {

  // Compute PFH features
  const float feature_radius = 0.03;
    ROS_INFO("Computing PFH features at the keypoints I found");
  compute_PFH_features_at_keypoints (downsampled1, normals1, keypoints1, feature_radius, descriptors1);
  compute_PFH_features_at_keypoints (downsampled2, normals2, keypoints2, feature_radius, descriptors2);
    ROS_INFO("DONE!");
  // Find feature correspondences
  std::vector<int> correspondences;
  std::vector<float> correspondence_scores;
    ROS_INFO("All that's left to do is to see how these two feature sets compare...");
  find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
    ROS_INFO("Done! And the results are:");
  // Print out ( number of keypoints / number of points )
  std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
            << "out of " << downsampled1->size () << " total points." << std::endl;

  std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
            << "out of " << downsampled2->size () << " total points." << std::endl;
  // Visualize the two point clouds and their feature correspondences


  std::cout << "CORRESPONDENCE \t\t SCORE" << std::endl;
  float good_scores = 0;
  for (size_t i = 0; i < correspondences.size (); ++i)
  {
    std::cout << correspondences[i] << " \t\t " << correspondence_scores[i] << std::endl;

    if(correspondence_scores[i] <= 0.35f) {
      good_scores+=1;
    }

  }

  good_scores = good_scores/correspondences.size();


  //float total_cor = good_scores/(float)keypoints1->size();
  std::cout << "CORRESPONDENCES: " << correspondences.size() << " KEYPOINTS: " << keypoints1->size() << std::endl;
  std::cout << "VISUAL FEATURE SCORE: " << good_scores << std::endl;
  //std::cout << "TOTAL SCORE: " << total_cor << std::endl;
  //return good_scores;
  std::cout << "OCTO SCORE: " << octo_score << std::endl;
  return octo_score*good_scores;

}
  return octo_score;
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
  octomap_msgs::Octomap out = convert_pcd_to_octomap(req.clouds,0.03f);
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
