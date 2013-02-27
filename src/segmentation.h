/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Feb 12, 2013
 *      Author: Kai Franke
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef sensor_msgs::PointCloud2 PointCloudROS;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main(int, char**);
inline void duration(bool identifier);

// if true filtered points will be removed from the point cloud, if false overwritten by NaN
bool keep_organized;

// Initialize Subscriber
ros::Subscriber sub;

// Do you want to filter the depth for a specific depth range?
bool filter_z;

// Range variables for z coordinaten in pointcloud, can be changed using parameters
double z_min, z_max;

// Distance threshold for plane
double threshold_plane;

// Range for cylinder radius
double radius_min, radius_max;

// Size of the downsampled voxel
double voxel_size;

// ROS message for filtered point cloud
PointCloudROS input_filtered;

// ROS message for point cloud output without the plane
PointCloudROS output;

// Declare the segmentation object for planes
pcl::SACSegmentation<PointType> seg_plane;

// Declare the filtering object for planes
pcl::ExtractIndices<PointType> extract_planes;

// Callback function when subscribed to point cloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);


