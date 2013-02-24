/*
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 */

#include "recognize.h"

void world_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	world               = PointCloud::Ptr    (new PointCloud    ());
	world_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	world_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	world_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *world);

	//compute normals
	cout << "... computing normals from world ..." << endl;
	norm_est_world.setInputCloud (world);
	norm_est_world.compute (*world_normals);


	//
	//  Downsample world to extract keypoints
	//
	uniform_sampling_world.setInputCloud (world);
	uniform_sampling_world.setRadiusSearch (world_ss_);
	uniform_sampling_world.compute (sampled_indices_world);
	pcl::copyPointCloud (*world, sampled_indices_world.points, *world_keypoints);
	std::cout << "World total points: " << world->size () << "; Selected Keypoints: " << world_keypoints->size () << std::endl;


  //
  // Extract descriptors
  //
	cout << "... extracting descriptors from world ..." << endl;
  descr_est_world.setInputCloud (world_keypoints);
  descr_est_world.setRadiusSearch (descr_rad_);
  descr_est_world.setInputNormals (world_normals);
  descr_est_world.setSearchSurface (world);
  descr_est_world.compute (*world_descriptors);

}

void object_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// check if world was already processed
	if (world_descriptors == NULL)
	{
		ROS_WARN("Received an object pointcloud before having a world pointcloud to compare");
		return;
	}

	object               = PointCloud::Ptr    (new PointCloud    ());
	object_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	object_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	object_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *object);

	//
	// compute normals
	//
	cout << "... computing normals from object ..." << endl;
	norm_est_object.setInputCloud (object);
	norm_est_object.compute (*object_normals);


	//
	//  Downsample object to extract keypoints
	//
	cout << "... downsampling object ..." << endl;
	uniform_sampling_object.setInputCloud (object);
	uniform_sampling_object.setRadiusSearch (object_ss_);
	uniform_sampling_object.compute (sampled_indices_object);
	pcl::copyPointCloud (*object, sampled_indices_object.points, *object_keypoints);
	std::cout << "Object total points: " << object->size () << "; Selected Keypoints: " << object_keypoints->size () << std::endl;


	//
	// Extract descriptors
	//
	cout << "... extracting descriptors from object ..." << endl;
	descr_est_object.setInputCloud (object_keypoints);
	descr_est_object.setRadiusSearch (descr_rad_);
	descr_est_object.setInputNormals (object_normals);
	descr_est_object.setSearchSurface (object);
	descr_est_object.compute (*object_descriptors);


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

		// convert Eigen matricies into ROS Pose message
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
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "object_tf"));
	
		while (ros::ok())
		{
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "object_tf"));
			sleep(1);
		}
  }

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "feature_detection");
	ros::NodeHandle nh;
	
	// Create a ROS subscriber for the world point cloud
	sub_world = nh.subscribe ("world_pointcloud", 1, world_cb);
	
	// Create a ROS subscriber for the object point cloud
	sub_object = nh.subscribe ("object_pointcloud", 1, object_cb);

  //  uSet parameters for normal computation
  norm_est_world.setKSearch (10);
  norm_est_object.setKSearch (10);

	//Algorithm params
	object_ss_ = 0.01;
	world_ss_ = 0.03;
	rf_rad_ = 0.015;
	descr_rad_ = 0.02;
	cg_size_ = 0.01;
	cg_thresh_ = 5.0;

	ros::spin();
	return 0;
}
