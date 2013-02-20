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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DesciptorCloud;

using namespace std;

// Timing variables
ros::Time start;
ros::Time stop;

// Initialize Subscriber
ros::Subscriber sub_world;
ros::Subscriber sub_object;

// Initialize Publisher for object coefficients in world
ros::Publisher obj_pose;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr model;
PointCloud::Ptr model_keypoints;
PointCloud::Ptr scene;
PointCloud::Ptr scene_keypoints;
NormalCloud::Ptr model_normals;
NormalCloud::Ptr scene_normals;
DesciptorCloud::Ptr model_descriptors;
DesciptorCloud::Ptr scene_descriptors;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
pcl::PointCloud<int> sampled_indices;
pcl::UniformSampling<PointType> uniform_sampling;
pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
