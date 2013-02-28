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
#include <object_recognition/Shot1344_bundle.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::SHOT352 SHOT352;
typedef pcl::SHOT1344 SHOT1344;
typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;
typedef object_recognition::Shot352_bundle Shot352Msg;
typedef object_recognition::Shot1344_bundle Shot1344Msg;
typedef sensor_msgs::PointCloud2 PointCloudROS;

using namespace std;

// Initialize Subscriber
ros::Subscriber sub_keypoint_world;
ros::Subscriber sub_keypoint_object;
ros::Subscriber sub_descriptors_object_shot352;
ros::Subscriber sub_descriptors_object_shot1344;
ros::Subscriber sub_descriptors_world_shot352;
ros::Subscriber sub_descriptors_world_shot1344;

// Publisher for debug output
ros::Publisher pub_object;
ros::Publisher pub_world;

//Algorithm params
double cg_size_, cg_thresh_, max_descr_dist_;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr object_keypoints;
PointCloud::Ptr world_keypoints;
DescriptorCloudShot352::Ptr object_descriptors_shot352;
DescriptorCloudShot352::Ptr world_descriptors_shot352;
DescriptorCloudShot1344::Ptr object_descriptors_shot1344;
DescriptorCloudShot1344::Ptr world_descriptors_shot1344;


void cluster(const pcl::CorrespondencesPtr &object_world_corrs);
