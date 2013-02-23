/*
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 */

#include "recognize.h"

void object_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	scene               = PointCloud::Ptr    (new PointCloud    ());
	scene_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	scene_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	scene_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *scene);

	//compute normals
	cout << "... computing normals from world ..." << endl;
	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);


	//
	//  Downsample world to extract keypoints
	//
	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
	std::cout << "World total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


  //
  // Extract descriptors
  //
	cout << "... extracting descriptors from world ..." << endl;
  descr_est.setInputCloud (scene_keypoints);
  descr_est.setRadiusSearch (descr_rad_);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

}

void world_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// check if world was already processed
	if (scene_descriptors == NULL)
	{
		ROS_WARN("Received an object pointcloud before having a world pointcloud to compare");
		return;
	}

	model               = PointCloud::Ptr    (new PointCloud    ());
	model_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	model_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	model_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *model);

	//
	// compute normals
	//
	cout << "... computing normals from object ..." << endl;
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);


	//
	//  Downsample object to extract keypoints
	//
	cout << "... downsampling object ..." << endl;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
	std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;


	//
	// Extract descriptors
	//
	cout << "... extracting descriptors from object ..." << endl;
	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);


	//
	//  Find Model-Scene Correspondences with KdTree
	//
	cout << "... finding correspondences ..." << endl;
	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
	
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud (model_descriptors);
		
	// For each scene keypoint descriptor
	// find nearest neighbor into the model keypoints descriptor cloud 
	// and add it to the correspondences vector
	for (size_t i = 0; i < scene_descriptors->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);
		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
		// add match only if the squared descriptor distance is less than 0.25 
		// SHOT descriptor distances are between 0 and 1 by design
		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) 
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
    

  //
  //  Actual Clustering
  //
	cout << "... clustering ..." << endl;    
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
    
	pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
	gc_clusterer.setGCSize (cg_size_);
	gc_clusterer.setGCThreshold (cg_thresh_);

	gc_clusterer.setInputCloud (model_keypoints);
	gc_clusterer.setSceneCloud (scene_keypoints);
	gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

	gc_clusterer.recognize (rototranslations, clustered_corrs);

    
  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
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
  norm_est.setKSearch (10);

	//Algorithm params
	model_ss_ = 0.01;
	scene_ss_ = 0.03;
	rf_rad_ = 0.015;
	descr_rad_ = 0.02;
	cg_size_ = 0.01;
	cg_thresh_ = 5.0;

	ros::spin();
	return 0;
}
