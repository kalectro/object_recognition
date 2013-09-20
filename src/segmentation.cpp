#include "segmentation.h"

using namespace std;

int main (int argc, char** argv)
{
	ros::init (argc, argv, "filter_pointcloud");
	ros::NodeHandle nh("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("input", 1, cloud_cb);
	// create publisher for filtered cloud
	pub = nh.advertise<PointCloudROS> ("output", 1);
	
	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000);
	
	// Extract the found plane to remove the table
	extract_planes.setNegative (true);

	// Spin
	ros::spin ();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	ros:: NodeHandle nh("~");
	// get all parameters from parameter server
	nh.param("threshold_plane", threshold_plane, 0.02);
	nh.param("keep_organized", keep_organized, false);
	nh.param("voxel_size", voxel_size, 0.01);
	nh.param("apply_voxel", apply_voxel, true);
	nh.param("z_min_distance", z_min, 0.0);
	nh.param("z_max_distance", z_max, 1.0);
	nh.param("filter_z", filter_z, false);

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

	// Create pass through point cloud for point filtering
	pcl::PassThrough<PointType> pt(false);

	// Create a pointcloud to store the z filtered cloud
	PointCloud::Ptr cloud_z_filtered (new PointCloud);

	// Create a pointcloud to store the voxeled pointcloud
	pcl::PCLPointCloud2::Ptr cloud_voxeled (new pcl::PCLPointCloud2);

  // Convert ROS message to PCL message
  PointCloud::Ptr pcl_cloud;
  pcl::PCLPointCloud2::Ptr pcl_cloud2;
  pcl_conversions::toPCL(*input, *pcl_cloud2);
  
  //
  // downscale the points
  //
  if(apply_voxel)
  {
    // Create the filtering object and downsample the dataset using the parameter leaf size
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_voxeled);
		sor.setLeafSize (voxel_size,voxel_size,voxel_size);
		sor.filter (*cloud_voxeled);
	}
	
	//
	// filter z values within a certain range set by parameters
	//
	// check if z values are supposed to be filtered (default: false)
	if(filter_z)
	{
		if(z_max-z_min <= 0)
		{
			ROS_WARN("Please make sure the parameter z_min_distance is smaller than z_max_distance");
			return;
		}
		// set parameters for z axis filtering
		if(apply_voxel)
    {
      pcl::fromPCLPointCloud2(*cloud_voxeled, *pcl_cloud);
		  pt.setInputCloud (pcl_cloud);
    }
		else
		{
		  pcl::fromPCLPointCloud2(*pcl_cloud2, *pcl_cloud);
			pt.setInputCloud (pcl_cloud);
    }
		pt.setKeepOrganized(keep_organized);
		pt.setFilterFieldName("z");
		pt.setFilterLimits(z_min, z_max);
		pt.filter(*cloud_z_filtered);
	}	

	// convert the message
	if (filter_z)
	{
	  cloud = cloud_z_filtered;
  }
	else if (apply_voxel)
	{
	  pcl::fromPCLPointCloud2(*cloud_voxeled, *cloud);
	}
	else
	{
	  pcl::fromPCLPointCloud2(*pcl_cloud2, *cloud);
	}

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
	extract_planes.filter (*cloud_no_plane);

	//
	// convert back to ROS message
	//
	pcl::toROSMsg(*cloud_no_plane, output);

	// fill in header
	output.header.stamp = ros::Time::now();
	output.header.frame_id = input->header.frame_id;

	pub.publish(output);
}

