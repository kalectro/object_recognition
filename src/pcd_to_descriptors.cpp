/*
 *
 *  Created on: Feb 24, 2013
 *      Author: Kai Franke
 */

#include "pcd_to_descriptors.h"

void toROSMsg(const DescriptorCloud &input, object_recognition::Shot352_bundle &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		std::copy(input[j].descriptor, input[j].descriptor + 352 , output.descriptors[j].descriptor.begin());
		std::copy(input[j].rf, input[j].rf + 9, output.descriptors[j].rf.begin());
	}

	// keep this if memcopy does not work
	//for (int j = 0 ; j < input.size() ; ++j)
	//{	
	//	for (int i = 0 ; i < 352 ; ++i)
	//		output.descriptors[j].descriptor[i] = input[j].descriptor[i];
	//	for (int i = 0 ; i < 9 ; ++i)
	//		output.descriptors[j].rf[i] = input[j].rf[i];
	//}
}

int main(int argc, char **argv)
{
	//
	// take care of the ROS stuff
	//
	ros::init(argc, argv, "cloud_descriptor");
	ros::NodeHandle nh("~");

	// Create a ROS publisher for the output model coefficients
	pub_keypoints   = nh.advertise<sensor_msgs::PointCloud2> ("keypoints", 1);
	pub_descriptors = nh.advertise<object_recognition::Shot352_bundle> ("descriptors", 1);

	// create objects
	output_keypoints = sensor_msgs::PointCloud2::Ptr (new sensor_msgs::PointCloud2);	
	output_descriptors = object_recognition::Shot352_bundle::Ptr (new object_recognition::Shot352_bundle);

	// If parameter pcd_path was not specified
	if (!nh.getParam("pcd_path", pcd_path))
	{
		ROS_ERROR("Private parameter pcd_path not found");
		ROS_ERROR("Usage: rosrun object_recognition pcd_descriptor _pcd_path:=/path/to/pcd");
		return -1;
	}

	//
	// create all neccessary objects
	//
	cloud               = PointCloud::Ptr     (new PointCloud    ());
	cloud_keypoints     = PointCloud::Ptr     (new PointCloud    ());
	cloud_normals       = NormalCloud::Ptr    (new NormalCloud   ());
	cloud_descriptors   = DescriptorCloud::Ptr(new DescriptorCloud());

	
	//
	// load the pcd into the point cloud
	//
	if (pcl::io::loadPCDFile (pcd_path, *cloud) == -1) 
	{
		PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
		return (-1);
	}

	// Algorithm params
	cloud_ss_ = 0.01;
	rf_rad_ = 0.015;
	descr_rad_ = 0.02;


	//
	// compute normals
	//
	cout << "... computing normals from cloud ..." << endl;
	norm_est_cloud.setKSearch (10);
	norm_est_cloud.setInputCloud (cloud);
	norm_est_cloud.compute (*cloud_normals);


	//
	//  Downsample cloud to extract keypoints
	//
	cout << "... downsampling cloud ..." << endl;
	uniform_sampling_cloud.setInputCloud (cloud);
	uniform_sampling_cloud.setRadiusSearch (cloud_ss_);
	uniform_sampling_cloud.compute (sampled_indices_cloud);
	pcl::copyPointCloud (*cloud, sampled_indices_cloud.points, *cloud_keypoints);
	std::cout << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;


	//
	// Extract descriptors
	//
	cout << "... extracting descriptors from cloud ..." << endl;
	descr_est_cloud.setInputCloud (cloud_keypoints);
	descr_est_cloud.setRadiusSearch (descr_rad_);
	descr_est_cloud.setInputNormals (cloud_normals);
	descr_est_cloud.setSearchSurface (cloud);
	descr_est_cloud.compute (*cloud_descriptors);

	//
	// Convert to ROS message
	//
	pcl::toROSMsg(*cloud_keypoints, *output_keypoints);
	toROSMsg(*cloud_descriptors, *output_descriptors);
	output_keypoints->header.frame_id = "/pcd_frame";

	// check if anyone subscribed
	if (pub_keypoints.getNumSubscribers() == 0)
	{
		ROS_WARN("No subscriber found");
		while (pub_keypoints.getNumSubscribers() == 0)
		{
			ros::Duration(1).sleep();
		}
		ROS_WARN("Subscriber now available, wait another 1 second");
		// give some time to set up connection
		ros::Duration(1).sleep();
	}


	//
	// Publish Keypoints and Descriptors
	//
	pub_keypoints.publish(*output_keypoints);
	pub_descriptors.publish(*output_descriptors);
	// wait before killing node to be able to transmit pointcloud
	ros::Duration(2).sleep();
	ros::spinOnce();

	return 0;
}
