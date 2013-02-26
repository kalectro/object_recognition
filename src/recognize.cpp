/*
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 */

#include "recognize.h"

void fromROSMsg(const object_recognition::Shot352_bundle &input,  DescriptorCloud &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
	{	
		std::copy(input.descriptors[j].descriptor.begin(), input.descriptors[j].descriptor.begin() + 352 , output[j].descriptor);
		std::copy(input.descriptors[j].rf.begin(), input.descriptors[j].rf.begin() + 9, output[j].rf);
	}
}

// Callback function when the world keypoints are received
void world_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	world_keypoints = PointCloud::Ptr (new PointCloud());
	pcl::fromROSMsg(*input, *world_keypoints);
}

// Callback function when the world descriptors are received
void world_descriptor_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	world_descriptors = DescriptorCloud::Ptr (new DescriptorCloud ());
	fromROSMsg(*input, *world_descriptors);
}

// Callback function when the object keypoints are received
void object_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	object_keypoints = PointCloud::Ptr (new PointCloud());
	pcl::fromROSMsg(*input, *object_keypoints);
}

// callback function when the object descriptors are received
// this will also trigger the recognition if all the other keypoints and descriptors have been received
void object_descriptor_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	// check if world was already processed
	if (world_descriptors == NULL)
	{
		ROS_WARN("Received object descriptors before having a world pointcloud to compare");
		return;
	}
	// check if the stored world descriptors can be assinged to the stored keypoints
	if ((int)world_keypoints->size() != (int)world_descriptors->size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the world. Number must be equal", (int)world_descriptors->size(), (int)world_keypoints->size());
		return;
	}
	// check if the received object descriptors can be assigned to the stored keypoints
	if ((int)object_keypoints->size() != (int)input->descriptors.size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the object. Number must be equal", (int)input->descriptors.size(), (int)object_keypoints->size());
		return;
	}
	
	object_descriptors = DescriptorCloud::Ptr (new DescriptorCloud ());
	fromROSMsg(*input, *object_descriptors);
	//
	//  Find Object-World Correspondences with KdTree
	//
	cout << "... finding correspondences ..." << endl;
	pcl::CorrespondencesPtr object_world_corrs (new pcl::Correspondences ());
	
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (object_descriptors);
		
	// For each world keypoint descriptor
	// find nearest neighbor into the object keypoints descriptor cloud 
	// and add it to the correspondences vector
	for (size_t i = 0; i < world_descriptors->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (world_descriptors->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch (world_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		// add match only if the squared descriptor distance is less than 0.25 
		// SHOT descriptor distances are between 0 and 1 by design
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			object_world_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << object_world_corrs->size () << std::endl;
    

  //
  //  Actual Clustering
  //
	cout << "... clustering ..." << endl;    
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
    
	pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
	gc_clusterer.setGCSize (cg_size_);
	gc_clusterer.setGCThreshold (cg_thresh_);

	gc_clusterer.setInputCloud (object_keypoints);
	gc_clusterer.setSceneCloud (world_keypoints);
	gc_clusterer.setModelSceneCorrespondences (object_world_corrs);

	gc_clusterer.recognize (rototranslations, clustered_corrs);

    
  //
  //  Output results
  //
  std::cout << "Object instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
    
    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
    
    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

		// convert Eigen matricies into ROS TF message
		tf::Vector3 object_offset;
		tf::Quaternion object_rotation;
		object_offset[0] = translation (0);
		object_offset[1] = translation (1);
		object_offset[2] = translation (2);
		// convert rotation matrix to quaternion
		Eigen::Quaternionf quaternion (rotation);
		object_rotation[0] = quaternion.x();
		object_rotation[1] = quaternion.y();
		object_rotation[2] = quaternion.z();
		object_rotation[3] = quaternion.w();

		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin (object_offset);
		transform.setRotation (object_rotation);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "object"));
	
		while (ros::ok())
		{
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "object"));
			ros::Duration(1).sleep();
		}
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "feature_detection");
	ros::NodeHandle nh;
	
	// Create a ROS subscriber for the object and world keypoints and descriptors
	sub_keypoint_object = nh.subscribe ("/cloud_descriptor/object/keypoints", 1, object_keypoint_cb);
	sub_descriptors_object = nh.subscribe ("/cloud_descriptor/object/descriptors", 1, object_descriptor_cb);
	sub_keypoint_world = nh.subscribe ("/cloud_descriptor/world/keypoints", 1, world_keypoint_cb);
	sub_descriptors_world = nh.subscribe ("/cloud_descriptor/world/descriptors", 1, world_descriptor_cb);

	//Algorithm params
	cg_size_ = 0.01;
	cg_thresh_ = 5.0;

	ros::spin();
	return 0;
}
