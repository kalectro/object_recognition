/*
 * recognize.h
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 * This program subscribes to a two point clouds and tries to match them
 */

#include "ros/ros.h"
#include <stdio.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <tf/transform_broadcaster.h>
#include <object_recognition/Shot352_bundle.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;

using namespace std;

// Initialize Subscriber
ros::Subscriber sub_world;
ros::Subscriber sub_descriptors;
ros::Subscriber sub_keypoint;

// Initialize Publisher for object tf frame in world
ros::Publisher pub_object_tf;

//Algorithm params
float object_ss_, world_ss_, rf_rad_, descr_rad_, cg_size_, cg_thresh_;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr object;
PointCloud::Ptr object_keypoints;
PointCloud::Ptr world;
PointCloud::Ptr world_keypoints;
NormalCloud::Ptr object_normals;
NormalCloud::Ptr world_normals;
DescriptorCloud::Ptr object_descriptors;
DescriptorCloud::Ptr world_descriptors;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est_world, norm_est_object;
pcl::PointCloud<int> sampled_indices_world, sampled_indices_object;
pcl::UniformSampling<PointType> uniform_sampling_object, uniform_sampling_world;
pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est_object, descr_est_world;
