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
#include <object_recognition/Shot352_bundle.h>
#include <object_recognition/Shot1344.h>
#include <object_recognition/Shot1344_bundle.h>
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
// hydro migration
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 SHOT352;
typedef pcl::SHOT1344 SHOT1344;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<SHOT352> DescriptorCloudShot352;
typedef pcl::PointCloud<SHOT1344> DescriptorCloudShot1344;
typedef sensor_msgs::PointCloud2 KeypointMsg;
typedef object_recognition::Shot352_bundle Shot352Msg;
typedef object_recognition::Shot1344_bundle Shot1344Msg;

using namespace std;

// Initialize subscriber and publisher for keypoints and descriptors
ros::Publisher pub_keypoints;
ros::Publisher pub_descriptors_Shot352;
ros::Publisher pub_descriptors_Shot1344;
ros::Subscriber sub;

string pcd_path;

//Algorithm params for Shot descriptor
double cloud_ss_, descr_rad_ ;
string output_frame;

// Point clouds for object, world and its normals, keypoints and descriptors
PointCloud::Ptr cloud;
PointCloud::Ptr cloud_keypoints;
NormalCloud::Ptr cloud_normals;
DescriptorCloudShot352::Ptr cloud_descriptors_shot352;
DescriptorCloudShot1344::Ptr cloud_descriptors_shot1344;
KeypointMsg::Ptr output_keypoints;
Shot352Msg::Ptr output_descriptors_shot352;
Shot1344Msg::Ptr output_descriptors_shot1344;

pcl::NormalEstimationOMP<PointType, NormalType> norm_est_cloud;
pcl::PointCloud<int> sampled_indices_cloud;
pcl::UniformSampling<PointType> uniform_sampling_cloud;
pcl::SHOTEstimationOMP<PointType, NormalType, SHOT352> descr_est_shot352;
pcl::SHOTColorEstimationOMP<PointType, NormalType, SHOT1344> descr_est_shot1344;

// function prototypes
void pointcloud_incoming (const sensor_msgs::PointCloud2ConstPtr& input);
void toROSMsg(const DescriptorCloudShot352 &input, Shot352Msg &output);
void toROSMsg(const DescriptorCloudShot1344 &input, Shot1344Msg &output);
