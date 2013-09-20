/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Feb 12, 2013
 *      Author: Kai Franke
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
// hydro migration
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef sensor_msgs::PointCloud2 PointCloudROS;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main(int, char**);
inline void duration(bool identifier);

// if true filtered points will be removed from the point cloud, if false overwritten by NaN
bool keep_organized;

// Initialize Subscriber and Publisher
ros::Subscriber sub;
ros::Publisher pub;

// Distance threshold for plane
double threshold_plane;

// Range for cylinder radius
double radius_min, radius_max;

// Size of the downsampled voxel
double voxel_size;
bool apply_voxel; // set to true if voxel filter should be applied

// Range variables for z coordinaten in pointcloud, can be changed using parameters
double z_min, z_max;
bool filter_z;	// set to true if z filter is supposed to be applied

// ROS message for point cloud output without the plane
PointCloudROS output;

// Declare the segmentation object for planes
pcl::SACSegmentation<PointType> seg_plane;

// Declare the filtering object for planes
pcl::ExtractIndices<PointType> extract_planes;

// Callback function when subscribed to point cloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);


