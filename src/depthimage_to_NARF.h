/*
 * pcd_to_descriptors.h
 *
 *  Created on: Mar 8, 2013
 *      Author: Kai Franke
 * This program subscribes to a point cloud and publishes its NARF keypoints and their descriptors
 */

// needed for NARF
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition/NARF.h>
#include <object_recognition/NARF_bundle.h>
#include <image_transport/image_transport.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::Narf36 NARF;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef pcl::PointCloud<NARF> DescriptorCloud;
typedef sensor_msgs::PointCloud2 KeypointMsg;
typedef object_recognition::NARF_bundle NARFMsg;

using namespace std;

// Initialize subscriber and publisher for keypoints and descriptors
ros::Publisher pub_keypoints;
ros::Publisher pub_descriptors;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

//Algorithm params for NARF descriptor
float angular_resolution, support_size;
bool setUnseenToMaxRange, rotation_invariant;
string output_frame;

// Point clouds for object, world and its normals, keypoints and descriptors
pcl::PointCloud<int> keypoint_indices;
KeypointMsg::Ptr output_keypoints;
NARFMsg output_descriptors_narf;
DescriptorCloud narf_descriptors;

// function prototypes
void range_image_incoming (const sensor_msgs::ImageConstPtr& imgMsgPtr);
void toROSMsg(const DescriptorCloud &input, NARFMsg &output);
