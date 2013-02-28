/*
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 */

#include "recognize.h"

// overloaded function to convert the custom ROS message into a PCL descriptor type
void fromROSMsg(const object_recognition::Shot352_bundle &input,  DescriptorCloudShot352 &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
	{	
		std::copy(input.descriptors[j].descriptor.begin(), input.descriptors[j].descriptor.begin() + 352 , output[j].descriptor);
		std::copy(input.descriptors[j].rf.begin(), input.descriptors[j].rf.begin() + 9, output[j].rf);
	}
}

void fromROSMsg(const object_recognition::Shot1344_bundle &input,  DescriptorCloudShot1344 &output)
{
	output.resize(input.descriptors.size());
	for (int j = 0 ; j < input.descriptors.size() ; ++j)
	{	
		std::copy(input.descriptors[j].descriptor.begin(), input.descriptors[j].descriptor.begin() + 1344 , output[j].descriptor);
		std::copy(input.descriptors[j].rf.begin(), input.descriptors[j].rf.begin() + 9, output[j].rf);
	}
}

// Callback function when the world keypoints are received
void world_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	world_keypoints = PointCloud::Ptr (new PointCloud());
	pcl::fromROSMsg(*input, *world_keypoints);
}

// Callback function when the object keypoints are received
void object_keypoint_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	object_keypoints = PointCloud::Ptr (new PointCloud());
	pcl::fromROSMsg(*input, *object_keypoints);
}

// Callback function when the world descriptors are received
void world_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	world_descriptors_shot352 = DescriptorCloudShot352::Ptr (new DescriptorCloudShot352 ());
	fromROSMsg(*input, *world_descriptors_shot352);
}

// Callback function when the world descriptors are received
void world_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input)
{
	world_descriptors_shot1344 = DescriptorCloudShot1344::Ptr (new DescriptorCloudShot1344 ());
	fromROSMsg(*input, *world_descriptors_shot1344);
}

// callback function when the object descriptors are received
// this will also trigger the recognition if all the other keypoints and descriptors have been received
void object_descriptor_shot352_cb (const object_recognition::Shot352_bundle::Ptr input)
{
	ros::NodeHandle nh;
	// check if world was already processed
	if (world_descriptors_shot352 == NULL)
	{
		ROS_WARN("Received object descriptors before having a world pointcloud to compare");
		return;
	}
	// check if the stored world descriptors can be assinged to the stored keypoints
	if ((int)world_keypoints->size() != (int)world_descriptors_shot352->size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the world. Number must be equal", (int)world_descriptors_shot352->size(), (int)world_keypoints->size());
		return;
	}
	// check if the received object descriptors can be assigned to the stored keypoints
	if ((int)object_keypoints->size() != (int)input->descriptors.size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the object. Number must be equal", (int)input->descriptors.size(), (int)object_keypoints->size());
		return;
	}
	
	// Debug output 
	ROS_INFO("Received %i descriptors for the world and %i for the object", (int)world_descriptors_shot352->size(), (int)input->descriptors.size());

	object_descriptors_shot352 = DescriptorCloudShot352::Ptr (new DescriptorCloudShot352 ());
	fromROSMsg(*input, *object_descriptors_shot352);
	//
	//  Find Object-World Correspondences with KdTree
	//
	cout << "... finding correspondences ..." << endl;
	pcl::CorrespondencesPtr object_world_corrs (new pcl::Correspondences ());
	
	pcl::KdTreeFLANN<SHOT352> match_search;
	match_search.setInputCloud (object_descriptors_shot352);

	// For each world keypoint descriptor
	// find nearest neighbor into the object keypoints descriptor cloud 
	// and add it to the correspondences vector
	for (size_t i = 0; i < world_descriptors_shot352->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (world_descriptors_shot352->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch (world_descriptors_shot352->at (i), 1, neigh_indices, neigh_sqr_dists);
		// add match only if the squared descriptor distance is less than 0.25 
		// SHOT descriptor distances are between 0 and 1 by design
		if(found_neighs == 1 && neigh_sqr_dists[0] < (float)max_descr_dist_) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			object_world_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << object_world_corrs->size () << std::endl;
	
	//
	// all keypoints and descriptors were found, no match the correspondences to the real object!
	//
	cluster(object_world_corrs);
}

// callback function when the object descriptors are received
// this will also trigger the recognition if all the other keypoints and descriptors have been received
void object_descriptor_shot1344_cb (const object_recognition::Shot1344_bundle::Ptr input)
{
	// check if world was already processed
	if (world_descriptors_shot1344 == NULL)
	{
		ROS_WARN("Received object descriptors before having a world pointcloud to compare");
		return;
	}
	// check if the stored world descriptors can be assinged to the stored keypoints
	if ((int)world_keypoints->size() != (int)world_descriptors_shot1344->size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the world. Number must be equal", (int)world_descriptors_shot1344->size(), (int)world_keypoints->size());
		return;
	}
	// check if the received object descriptors can be assigned to the stored keypoints
	if ((int)object_keypoints->size() != (int)input->descriptors.size())
	{
		ROS_WARN("Received %i descriptors and %i keypoints for the object. Number must be equal", (int)input->descriptors.size(), (int)object_keypoints->size());
		return;
	}
	
	object_descriptors_shot1344 = DescriptorCloudShot1344::Ptr (new DescriptorCloudShot1344 ());
	fromROSMsg(*input, *object_descriptors_shot1344);

	// Debug output 
	ROS_INFO("Received %i descriptors for the world and %i for the object", (int)world_descriptors_shot1344->size(), (int)input->descriptors.size());

	//
	//  Find Object-World Correspondences with KdTree
	//
	cout << "... finding correspondences ..." << endl;
	pcl::CorrespondencesPtr object_world_corrs (new pcl::Correspondences ());
	
	pcl::KdTreeFLANN<SHOT1344> match_search;
	match_search.setInputCloud (object_descriptors_shot1344);
		
	// For each world keypoint descriptor
	// find nearest neighbor into the object keypoints descriptor cloud 
	// and add it to the correspondences vector
	for (size_t i = 0; i < world_descriptors_shot1344->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (world_descriptors_shot1344->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch (world_descriptors_shot1344->at (i), 1, neigh_indices, neigh_sqr_dists);
		// add match only if the squared descriptor distance is less than 0.25 
		// SHOT descriptor distances are between 0 and 1 by design
		if(found_neighs == 1 && neigh_sqr_dists[0] < (float)max_descr_dist_) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			object_world_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << object_world_corrs->size () << std::endl;
	
	//
	// all keypoints and descriptors were found, no match the correspondences to the real object!
	//
	cluster(object_world_corrs);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_recognition");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	
	// Create a ROS subscriber for the object and world keypoints and descriptors
	sub_keypoint_object = nh.subscribe ("/object_recognition/object/keypoints", 1, object_keypoint_cb);
	sub_keypoint_world  = nh.subscribe ("/object_recognition/world/keypoints",  1, world_keypoint_cb );
	sub_descriptors_object_shot352 = nh.subscribe ("/object_recognition/object/descriptors/Shot352" , 1, object_descriptor_shot352_cb);
	sub_descriptors_world_shot352  = nh.subscribe ("/object_recognition/world/descriptors/Shot352"  , 1, world_descriptor_shot352_cb );
	sub_descriptors_object_shot1344= nh.subscribe ("/object_recognition/object/descriptors/Shot1344", 1, object_descriptor_shot1344_cb);
	sub_descriptors_world_shot1344 = nh.subscribe ("/object_recognition/world/descriptors/Shot1344" , 1, world_descriptor_shot1344_cb );

	pub_object = nh.advertise<PointCloudROS> ("correspondences/object/clustered", 1);	
	pub_world = nh.advertise<PointCloudROS> ("correspondences/world/clustered", 1);
	pub_object2 = nh.advertise<PointCloudROS> ("correspondences/object", 1);	
	pub_world2 = nh.advertise<PointCloudROS> ("correspondences/world", 1);

	// Get the parameter for the maximum descriptor distance 
	nh_param.param<double>("maximum_descriptor_distance" , max_descr_dist_ , 0.25 );
	nh_param.param<double>("cg_size" , cg_size_ , 0.01 );
	nh_param.param<double>("cg_thresh", cg_thresh_, 5.0);

	ros::spin();
	return 0;
}


void cluster(const pcl::CorrespondencesPtr &object_world_corrs)
{
	//
	// Debug output
	//
	PointCloud correspondence_object;
	PointCloud correspondence_world;
	cout << object_world_corrs->size () << endl;
	for (int j = 0; j < object_world_corrs->size (); ++j)
  {
    PointType& model_point = object_keypoints->at(object_world_corrs->at(j).index_query);
    PointType& scene_point = world_keypoints->at(object_world_corrs->at(j).index_match);
		correspondence_object.push_back(model_point);
		correspondence_world.push_back(scene_point);
  }

	PointCloudROS pub_me_object2;
	PointCloudROS pub_me_world2;
	toROSMsg(correspondence_object, pub_me_object2);
	toROSMsg(correspondence_world, pub_me_world2);
	pub_me_object2.header.frame_id = "/object";
	pub_me_world2.header.frame_id = "/world";
	pub_object2.publish(pub_me_object2);
	pub_world2.publish(pub_me_world2);

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

		//
		// Debug output
		//
		PointCloud correspondence_object_cluster;
		PointCloud correspondence_world_cluster;
		
		for (int j = 0; j < clustered_corrs[0].size (); ++j)
    {
      PointType& model_point = object_keypoints->at(clustered_corrs[0][j].index_query);
      PointType& scene_point = world_keypoints->at(clustered_corrs[0][j].index_match);
			correspondence_object_cluster.push_back(model_point);
			correspondence_world_cluster.push_back(scene_point);
    }

		PointCloudROS pub_me_object;
		PointCloudROS pub_me_world;
		toROSMsg(correspondence_object_cluster, pub_me_object);
		toROSMsg(correspondence_world_cluster, pub_me_world);
		pub_me_object.header.frame_id = "/object";
		pub_me_world.header.frame_id = "/world";
		pub_object.publish(pub_me_object);
		pub_world.publish(pub_me_world);


		// only publish the first correspondence
		break;
  }

	for (int i = 0; i < rototranslations.size (); ++i)
	{
		cout << "Instance "<< i << " had " << clustered_corrs[i].size () << " correspondences" << endl;
	}
}
