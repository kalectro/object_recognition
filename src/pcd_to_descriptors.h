/*
 * pcd_to_descriptors.h
 *
 *  Created on: Feb 24, 2013
 *      Author: Kai Franke
 * This program reads a pcd file and publishes its keypoint descriptors
 */

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition/Shot352.h>
#include <stdio.h>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<DescriptorType> DesciptorCloud;

using namespace std;

// Initialize Publisher for keypoints and descriptors
ros::Publisher pub_keypoints, pub_descriptors;

string pcd_path;

//Algorithm params
float cloud_ss_,rf_rad_, descr_rad_ ;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr cloud;
PointCloud::Ptr cloud_keypoints;
NormalCloud::Ptr cloud_normals;
DesciptorCloud::Ptr cloud_descriptors;
sensor_msgs::PointCloud2::Ptr output_keypoints;
object_recognition::Shot352::Ptr output_descriptors;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est_cloud;
pcl::PointCloud<int> sampled_indices_cloud;
pcl::UniformSampling<PointType> uniform_sampling_cloud;
pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est_cloud;
