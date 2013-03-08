/*
 *
 *  Created on: Mar 8, 2013
 *      Author: Kai Franke
 */

#include "depthimage_to_NARF.h"

// overloaded function to convert the PCL descriptor type to a custom ROS message
void toROSMsg(const DescriptorCloud &input, NARFMsg &output)
{
	output.descriptors.resize(input.size());
	for (int j = 0 ; j < input.size() ; ++j)
	{	
		output.descriptors[j].x = input[j].x;
		output.descriptors[j].y = input[j].y;
		output.descriptors[j].z = input[j].z;
		output.descriptors[j].roll = input[j].roll;
		output.descriptors[j].pitch = input[j].pitch;
		output.descriptors[j].yaw = input[j].yaw;
		std::copy(input[j].descriptor, input[j].descriptor + 36 , output.descriptors[j].descriptor.begin());
	}
}

int main (int argc, char** argv)
{
	//
	// take care of the ROS stuff
	//
	ros::init(argc, argv, "NARF_descriptor");
	ros::NodeHandle nh("~");

	// Create a ROS subscriber for the input range image
	cout << "Setting up image transport...";
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber range_image_sub;
	range_image_sub = it.subscribe("/depth_image", 1, &range_image_incoming);
	cout << "done" << endl;

	// Create a ROS publisher for the output model coefficients
	pub_keypoints = nh.advertise<KeypointMsg> ("keypoints", 1);
	pub_descriptors = nh.advertise<NARFMsg> ("/object_recognition/descriptors/NARF", 1);

	angular_resolution = -1.0;
	support_size = 0.2f;
	coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	setUnseenToMaxRange = false;
	rotation_invariant = true;

	ros::spin();
	return 0;
}

void range_image_incoming (const sensor_msgs::ImageConstPtr& imgMsgPtr)
{
	// ---------------------------------------------
	// -------Convert ROS image to range image------
	// ---------------------------------------------
	const uint16_t* depthImage = reinterpret_cast<const uint16_t*>(&imgMsgPtr->data[0]);
	pcl::RangeImagePlanar range_image_;  
	range_image_.setDepthImage(depthImage,imgMsgPtr->width, imgMsgPtr->height, \
				(imgMsgPtr->width)/2, (imgMsgPtr->height)/2, 600.0, 600.0, angular_resolution);
	range_image_.setUnseenToMaxRange();

  // --------------------------------	
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image_);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  narf_keypoint_detector.compute (keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  
  // ------------------------------------------------------
  // -----Extract NARF descriptors for interest points-----
  // ------------------------------------------------------
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];
  pcl::NarfDescriptor narf_descriptor (&range_image_, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  narf_descriptor.compute (narf_descriptors);
  cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";

	// convert to ROS message and publish
	toROSMsg(narf_descriptors,output_descriptors_narf);
	pub_descriptors.publish(output_descriptors_narf);
}
