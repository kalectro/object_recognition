/*
 *
 *  Created on: Feb 24, 2013
 *      Author: Kai Franke
 */

#include "pcd_to_descriptors.h"

// overloaded function to convert the PCL descriptor type to a custom ROS message
void toROSMsg(const DescriptorCloudShot352 &input, Shot352Msg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		std::copy(input[j].descriptor, input[j].descriptor + 352 , output.descriptors[j].descriptor.begin());
		std::copy(input[j].rf, input[j].rf + 9, output.descriptors[j].rf.begin());
	}
}
void toROSMsg(const DescriptorCloudShot1344 &input, Shot1344Msg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		std::copy(input[j].descriptor, input[j].descriptor + 1344 , output.descriptors[j].descriptor.begin());
		std::copy(input[j].rf, input[j].rf + 9, output.descriptors[j].rf.begin());
	}
}

int main(int argc, char **argv)
{
	//
	// take care of the ROS stuff
	//
	ros::init(argc, argv, "pcd_descriptor");
	ros::NodeHandle nh("~");

	// Create a ROS publisher for the output model coefficients
	pub_keypoints = nh.advertise<KeypointMsg> ("keypoints", 1);
	pub_descriptors_Shot352 = nh.advertise<Shot352Msg> ("descriptors/Shot352", 1);
	pub_descriptors_Shot1344= nh.advertise<Shot1344Msg>("descriptors/Shot1344",1);

	// create objects
	output_keypoints = KeypointMsg::Ptr (new KeypointMsg);	
	output_descriptors_shot352 = Shot352Msg ::Ptr (new Shot352Msg );
	output_descriptors_shot1344= Shot1344Msg::Ptr (new Shot1344Msg);

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
	cloud               			= PointCloud::Ptr     				(new PointCloud    ());
	cloud_keypoints     			= PointCloud::Ptr     				(new PointCloud    ());
	cloud_normals       			= NormalCloud::Ptr						(new NormalCloud   ());
	cloud_descriptors_shot352 = DescriptorCloudShot352::Ptr	(new DescriptorCloudShot352());
	cloud_descriptors_shot1344= DescriptorCloudShot1344::Ptr(new DescriptorCloudShot1344());

	
	//
	// load the pcd into the point cloud
	//
	if (pcl::io::loadPCDFile (pcd_path, *cloud) == -1) 
	{
		PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
		return (-1);
	}

	// retrieve all parameter variables from server or set to a default value
	nh.param<double>("cloud_ss" , cloud_ss_ , 0.01 );
	nh.param<double>("descr_rad", descr_rad_, 0.02 );
	nh.param<string>("output_frame", output_frame, "pcd_frame");

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
	descr_est_shot352.setInputCloud (cloud_keypoints);
	descr_est_shot352.setRadiusSearch (descr_rad_);
	descr_est_shot352.setInputNormals (cloud_normals);
	descr_est_shot352.setSearchSurface (cloud);
	descr_est_shot352.compute (*cloud_descriptors_shot352);

	descr_est_shot1344.setInputCloud (cloud_keypoints);
	descr_est_shot1344.setRadiusSearch (descr_rad_);
	descr_est_shot1344.setInputNormals (cloud_normals);
	descr_est_shot1344.setSearchSurface (cloud);
	descr_est_shot1344.compute (*cloud_descriptors_shot1344);

	//
	// Convert to ROS message
	//
	pcl::toROSMsg(*cloud_keypoints, *output_keypoints);
	toROSMsg(*cloud_descriptors_shot352, *output_descriptors_shot352);
	toROSMsg(*cloud_descriptors_shot1344, *output_descriptors_shot1344);
	output_keypoints->header.frame_id = output_frame;

	// check if anyone subscribed to keypoints
	if (pub_keypoints.getNumSubscribers() == 0)
	{
		ROS_WARN("No keypoint subscriber found");
		while (pub_keypoints.getNumSubscribers() == 0 && ros::ok())
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
	pub_descriptors_Shot352.publish(*output_descriptors_shot352);
	pub_descriptors_Shot1344.publish(*output_descriptors_shot1344);
	// wait before killing node to be able to transmit pointcloud
	ros::Duration(2).sleep();
	ros::spinOnce();

	return 0;
}
