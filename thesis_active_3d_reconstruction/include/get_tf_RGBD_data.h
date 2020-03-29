/* get_tf_RGBD_data.h
Andrew Yuanheng Cao 10 March 2020
Adapted includes from ORB-SLAM2/RGBDNode.h. */
#ifndef get_tf_RGBD_data_h
#define get_tf_RGBD_data_h

#include <string>
#include <thread>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <sys/resource.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <ros/time.h>
#include <rosbag/bag.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <tf2_msgs/TFMessage.h> // For TF transform object
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <dynamic_reconfigure/server.h>

// Make the GetData class public and available in the header so callback it is not erased after execution
class GetData
{

	public:
		void ImageCallback(const sensor_msgs::ImageConstPtr& image,const tf2_msgs::TFMessageConstPtr& tf);
};


#endif