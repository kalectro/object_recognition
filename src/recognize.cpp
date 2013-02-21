/*
 *
 *  Created on: Feb 18, 2013
 *      Author: Kai Franke
 */

#include "recognize.h"

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


void world_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	scene               = PointCloud::Ptr    (new PointCloud    ());
	scene_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	scene_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	scene_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *scene);

	//compute normals
	norm_est.setInputCloud (scene);
	norm_est.compute (*scene_normals);

	//
	//  Downsample world to extract keypoints
	//
	uniform_sampling.setInputCloud (scene);
	uniform_sampling.setRadiusSearch (scene_ss_);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
	std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
cout << "asdf1" << endl;
    //
    // Extract descriptors
    //
    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);
cout << "asdf2" << endl;
}

void object_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	model               = PointCloud::Ptr    (new PointCloud    ());
	model_keypoints     = PointCloud::Ptr    (new PointCloud    ());
	model_normals       = NormalCloud::Ptr   (new NormalCloud   ());
	model_descriptors   = DesciptorCloud::Ptr(new DesciptorCloud());

	pcl::fromROSMsg(*input, *model);
	cout << "asdf3" << endl;

	//
  //  Set up resolution invariance
  //
	float resolution = static_cast<float> (computeCloudResolution (model));
	if (resolution != 0.0f)
	{
		model_ss_   *= resolution;
		scene_ss_   *= resolution;
		rf_rad_     *= resolution;
		descr_rad_  *= resolution;
		cg_size_    *= resolution;
	}

	std::cout << "Model resolution:       " << resolution << std::endl;
	std::cout << "Model sampling size:    " << model_ss_ << std::endl;
	std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
	std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
	std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
	std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;



	// compute normals
	norm_est.setInputCloud (model);
	norm_est.compute (*model_normals);
cout << "asdf4" << endl;
	//
	//  Downsample object to extract keypoints
	//
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
	std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
cout << "asdf5" << endl;
	//
	// Extract descriptors
	//
	descr_est.setInputCloud (model_keypoints);
	descr_est.setInputNormals (model_normals);
	descr_est.setSearchSurface (model);
	descr_est.compute (*model_descriptors);
	
	//
	//  Find Model-Scene Correspondences with KdTree
	//
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
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    
	pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
	gc_clusterer.setGCSize (cg_size_);
	gc_clusterer.setGCThreshold (cg_thresh_);

	gc_clusterer.setInputCloud (model_keypoints);
	gc_clusterer.setSceneCloud (scene_keypoints);
	gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

	//gc_clusterer.cluster (clustered_corrs);
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

    
    //  Set parameters for normal computation
    norm_est.setKSearch (10);

    //  Compute Descriptor for keypoints
    descr_est.setRadiusSearch (descr_rad_);

    ros::spin();
	return 0;
}
