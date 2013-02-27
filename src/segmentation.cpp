#include "segmentation.h"

using namespace std;

int main (int argc, char** argv)
{
	ros::init (argc, argv, "object_recognition");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("pointcloud", 1, cloud_cb);
	
	// set all available variables to a default value to make them visible to the user
	nh.setParam("/ros_benchmark/pcl_segmentation/plane/threshold", 0.02);
	nh.setParam("/ros_benchmark/pcl_segmentation/keep_organized", true);
	nh.setParam("/ros_benchmark/pcl_segmentation/voxel_size", 0.01);

	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000);
	
	// Extract the found plane to remove the table
	extract_planes.setNegative (true);

	// get verbosity for pcd output
	nh_private.param<bool>("output_pcd", output_pcd, false);

	// Spin
	ros::spin ();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ros:: NodeHandle nh;
	// get all parameters from parameter server
	nh.getParam("plane/threshold", threshold_plane);
	nh.getParam("keep_organized", keep_organized);
	nh.getParam("voxel_size", voxel_size);

	// Construct point cloud to work with
	PointCloud::Ptr cloud (new PointCloud);

	// Construct point cloud after plane removal
	PointCloud::Ptr cloud_no_plane (new PointCloud);

	// construct coefficients for plane
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

	// constructor for point found as part of planar surface
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);

	// Create KdTree needed for normal estimation
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

	// Create a pointcloud to store the downsampled point cloud
	PointCloudROS::Ptr input_voxeled (new PointCloudROS);


	//
	// measure time for downscaling the points
	//
	// Create the filtering object and downsample the dataset using the parameter leaf size
	pcl::VoxelGrid<PointCloudROS> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (voxel_size,voxel_size,voxel_size);
	sor.filter (*input_voxeled);

	// convert the message
	pcl::fromROSMsg (input_filtered, *cloud);

	// set maximal distance from point to planar surface to be identified as plane
	seg_plane.setDistanceThreshold (threshold_plane);
	seg_plane.setInputCloud (cloud);
	seg_plane.segment (*inliers_plane, *coefficients_plane);


	// 
	// remove plane from point cloud
	// 
	extract_planes.setInputCloud(cloud);
	extract_planes.setIndices (inliers_plane);
	extract_planes.setKeepOrganized(keep_organized);
	// limit size for radius in meters
	seg_cylinder.setRadiusLimits (radius_min, radius_max);
	extract_planes.filter (*cloud_no_plane);

	//
	// convert back to ROS message
	//
	pcl::toROSMsg(*cloud_no_plane, output);

	// fill in header
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "pointcloud_frame";

}

