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
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

using namespace std;

// Timing variables
ros::Time start;
ros::Time stop;

// Initialize Subscriber
ros::Subscriber sub;

// Initialize Publisher for object coefficients in world
ros::Publisher obj_pose;


