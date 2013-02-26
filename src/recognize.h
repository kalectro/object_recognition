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
typedef pcl::SHOT352 DescriptorType;
typedef pcl::PointCloud<DescriptorType> DescriptorCloud;

using namespace std;

// Initialize Subscriber
ros::Subscriber sub_descriptors_object;
ros::Subscriber sub_keypoint_object;
ros::Subscriber sub_descriptors_world;
ros::Subscriber sub_keypoint_world;

// Initialize Publisher for object tf frame in world
ros::Publisher pub_object_tf;

//Algorithm params
float cg_size_, cg_thresh_;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr object_keypoints;
PointCloud::Ptr world_keypoints;
DescriptorCloud::Ptr object_descriptors;
DescriptorCloud::Ptr world_descriptors;
