// This node will read a pcd file and publish the point cloud

// Author: Kai Franke
// Date : Feb 16th 2013

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

string pcd_path, pcd_path_temp;

//Initialize Publisher
ros::Publisher pub;

// Parameters used if not available on parameter server
static const string PCD_PATH = "src/ros_benchmark/pcl_segmentation/pcds/benchmark.pcd";

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "create_pointcloud");
	ros::NodeHandle nh("~");

	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

	// Create a pointcloud to load the pcd file to
	sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);	

	// Set publishing frequency
	ros::Rate loop_rate(1);

	while (ros::ok())
	{
		// get pcd path from parameter server
		if (nh.getParam("pcd_path", pcd_path_temp))
		{
			ROS_DEBUG("Found pcd path %s on server", pcd_path_temp.c_str());
		}
		else // parameter for pcd path does not exist
		{
			pcd_path_temp = PCD_PATH;
			ROS_DEBUG("Found no pcd path on server, trying %s", pcd_path.c_str());
		}

		// read pcd only if changed
		if (pcd_path_temp != pcd_path)
		{
			pcd_path = pcd_path_temp;
			// load the pcd into the point cloud
			if (pcl::io::loadPCDFile (pcd_path, *cloud) == -1) 
			{
				PCL_ERROR ("Couldn't read file %s \n", pcd_path.c_str());
				return (-1);
			}
		}

		// Fill in the header
		(*cloud).header.stamp = ros::Time::now();
    	(*cloud).header.frame_id = "pointcloud_frame";

		// Publish the model coefficients
		pub.publish (*cloud);

		// Spin
		ros::spinOnce ();
		loop_rate.sleep();
	}
}
