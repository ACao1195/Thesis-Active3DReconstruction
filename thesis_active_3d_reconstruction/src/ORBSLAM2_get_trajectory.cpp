// ORBSLAM2_get_trajectory.cpp

// This program reads the Pose output from ORBSLAM2 (message type PoseStamped) and combines them into an array to form a nav_msgs/Path message.
	// Subscribed line: /orb_slam2_rgbd/pose
	// Published lineL /orb_slam2_rgbd/trajectory


#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <signal.h>
#include <fstream>
#include <array>
#include <cmath>

// Use Eigen3 library for vector math
#include <Eigen/Dense>

// Use Boost QVM library for Quaternion calculations
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <boost/qvm/quat_access.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace Eigen;
using namespace boost::qvm;

// Rotate by this much
	// TODO Improvement: Make this a launch arg, allow rotation in other directions
double theta = -0.47; // = 26.92 deg

// Vector3f camOffset(0.035, 0.0003, 0.053); // x,y,z camera offset from end effector
Vector3f camOffset;

// File to save trajectory to - is located in .ros folder
string save_path = "trajectory.bag";

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Define paths as global variables for simplicity
nav_msgs::Path path_ORBSLAM2; 
nav_msgs::Path path_arm;

// Have publisher as global variable
ros::Publisher ORBSLAM2_traj_pub;
ros::Publisher arm_traj_pub; 

// Variables for determining arm position
int firstTF = 0;
float xOffset;
float yOffset;
float zOffset;

int n_ORBSLAM2_pose = 1;
int n_arm_pose = 1;

float xRotOffset;
float yRotOffset;
float zRotOffset;
float wRotOffset;

// Debug check
int n = 0;

void mySigIntHandler(int sig){
	// Save the trajectory to a bag
	rosbag::Bag bag;
	bag.open(save_path, rosbag::bagmode::Write);

	bag.write("/orb_slam2_rgbd/trajectory", ros::Time::now(), path_ORBSLAM2);
	bag.write("/tf/trajectory", ros::Time::now(), path_arm);
	bag.close();

	g_request_shutdown = 1;
}

void pose_ORBSLAM2_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	float xPosInit, yPosInit, zPosInit;

	// Wait for the first arm tf message to get the offset for calculations
	if(firstTF == 0) {
		return;
	}

	// Verify pose received
	ROS_INFO_STREAM("Received ORBSLAM2 pose: " << n_ORBSLAM2_pose);
	n_ORBSLAM2_pose++;

	geometry_msgs::PoseStamped pose;

	path_ORBSLAM2.header = msg->header;
	pose.header = msg->header;

	// Use this only if the object is flat, i.e. aligned with the z-axis
	// pose.pose = msg->pose;
	
	// Need to rotate the trajectory so that it is flat with the z-axis
	xPosInit = msg->pose.position.x;
	yPosInit = msg->pose.position.y;
	zPosInit = msg->pose.position.z;

//	ROS_INFO_STREAM("ORBSLAM2 raw position: " << xPosInit << " y: " << yPosInit << " z: " << zPosInit);

	double cosTheta = cos(theta);
	double sinTheta = sin(theta);

	// Rotate about y-axis and shift to position of arm
	// Flip x and z as they are upside down, and offset by starting position of tf
	pose.pose.position.x = xOffset - (xPosInit * cosTheta + zPosInit * sinTheta);
	pose.pose.position.y = yOffset + (yPosInit);
	pose.pose.position.z = zOffset - (zPosInit * cosTheta - xPosInit * sinTheta);

//	ROS_INFO_STREAM("ORBSLAM2 Pose.pose.position.y = " << pose.pose.position.y);

	// Apply 180 degree rotation about y-axis
	boost::qvm::quat<double> ORBSLAMquat = {msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z};

	// Rotation by 180 deg in y-axis (account for skew correction)
	boost::qvm::quat<double> rotQuat = boost::qvm::roty_quat(3.14159f + theta);

	// Rotate ORBSLAMquat by rotQuat
	boost::qvm::quat<double> finalQuat = boost::qvm::operator*(rotQuat, ORBSLAMquat);

	pose.pose.orientation.x = boost::qvm::X(finalQuat);
	pose.pose.orientation.y = boost::qvm::Y(finalQuat);
	pose.pose.orientation.z = boost::qvm::Z(finalQuat);
	pose.pose.orientation.w = boost::qvm::S(finalQuat);

//	pose.pose.orientation = msg->pose.orientation;

	// Debug: Print header of msg
	// ROS_INFO_STREAM("\nReceived pose: " << msg->header);

	// path_ORBSLAM2.header = msg->header;
	path_ORBSLAM2.poses.push_back(pose); // Add new pose to path array

	// Debug - print path - does not work
	// ROS_INFO_STREAM("\nPath: " << path_ORBSLAM2.poses[0].pose);
	// n = n + 1;

	// Can publish directly to view in rviz or similar
	//	traj_pub.publish(path_ORBSLAM2);
	ORBSLAM2_traj_pub.publish(path_ORBSLAM2);

}

void pose_arm_Callback(/*const sensor_msgs::ImageConstPtr& image,*/ const tf2_msgs::TFMessage::ConstPtr& msg){
//	ROS_INFO_STREAM("Got tf\n"); // Debug line

// Verify pose received
	ROS_INFO_STREAM("Received arm pose: " << n_arm_pose);
	n_arm_pose++;

	geometry_msgs::TransformStamped single_tf;
	geometry_msgs::PoseStamped pose;

	// Get tf from last seen value
	single_tf = msg->transforms.back();

	// z cannot be 0 (in table), so break if blank data received
	if(single_tf.transform.translation.z == 0){
		return;
	}


	// Only grab tf frames related to child frame "tool0_controller"
	if(single_tf.child_frame_id == "tool0_controller"){

	//	ROS_INFO_STREAM("Got tf for tool0_controller\n"); // Debug line
		path_arm.header = single_tf.header;
		path_arm.header.frame_id = "map"; // To match in line with ORBSLAM2 naming conventions

		pose.header = single_tf.header;
		pose.header.frame_id = "map";

		// Debug test - override quaternion rotation
//		Vector3f quatXYZ(0.7071068, 0, 0);
//		float quatW = 0.7071068;

		// Place quaternion into a vector, with w value trated as scalar
		Vector3f quatXYZ(single_tf.transform.rotation.x, single_tf.transform.rotation.y, single_tf.transform.rotation.z);
		float quatW = single_tf.transform.rotation.w;

		// Calculate offset of camera, rotated according to the end effector
		// Rotated vector v' = 2 * dot(u,v) * u + (s^2 - dot(u,u)) * v + 2s (u x v) - where u = [q.x,q.y,q.z]
		Vector3f camOffsetRotated = 2.0f * (camOffset.dot(quatXYZ)) * quatXYZ 
					+ (quatW * quatW - quatXYZ.dot(quatXYZ)) * camOffset 
					+ 2.0f * quatW * quatXYZ.cross(camOffset);

//		ROS_INFO_STREAM("\ncamOffsetRotated values: " << camOffsetRotated);


		// Have to reassign translation/position as they are different formats
		pose.pose.position.x = single_tf.transform.translation.x + camOffsetRotated(0);
		pose.pose.position.y = single_tf.transform.translation.y + camOffsetRotated(1);
		pose.pose.position.z = single_tf.transform.translation.z + camOffsetRotated(2);

//		ROS_INFO_STREAM("arm Pose.pose.position.y = " << pose.pose.position.y);

		// Get offsets from first TF message
		if(firstTF == 0) {
			firstTF = 1;
			xOffset = single_tf.transform.translation.x + camOffsetRotated(0);
			yOffset = single_tf.transform.translation.y + camOffsetRotated(1);
			zOffset = single_tf.transform.translation.z + camOffsetRotated(2);

			xRotOffset = single_tf.transform.rotation.x;
			yRotOffset = single_tf.transform.rotation.y;
			zRotOffset = single_tf.transform.rotation.z;
			wRotOffset = single_tf.transform.rotation.w;

			// Debug: Print starting offsets
			ROS_INFO_STREAM("\n Offsets are: \n x:"<< xOffset << "\n y:" << yOffset << "\n z:" << zOffset);
		}

		// Place orientation into quaternion
		boost::qvm::quat<double> armQuat = {single_tf.transform.rotation.w, single_tf.transform.rotation.x, single_tf.transform.rotation.y, single_tf.transform.rotation.z};

		// Remember qvm::quat is in form {w, x, y, z}

		// Rotate about z-axis 90 deg
		boost::qvm::quat<double> rotQuat = boost::qvm::rotz_quat(1.570796f);

		// Rotate about rotQuat
		boost::qvm::quat<double> finalQuat = boost::qvm::operator*(armQuat, rotQuat);

		pose.pose.orientation.x = boost::qvm::X(finalQuat);
		pose.pose.orientation.y = boost::qvm::Y(finalQuat);
		pose.pose.orientation.z = boost::qvm::Z(finalQuat);
		pose.pose.orientation.w = boost::qvm::S(finalQuat);

		path_arm.poses.push_back(pose); // Add new pose to path array

		arm_traj_pub.publish(path_arm); // Publish pose
	}
}

int main(int argc, char **argv)
{

	// Initialise ROS, and override SIGINT handler (what happens with ctrl+c) with my own code
	ros::init(argc, argv, "ORBSLAM2_get_trajectory", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);

	ros::NodeHandle traj_nh;

	// Retreive camera offset values from launch file
	float camOffsetX, camOffsetY, camOffsetZ;
	traj_nh.getParam("camOffsetX", camOffsetX);
	traj_nh.getParam("camOffsetY", camOffsetY);
	traj_nh.getParam("camOffsetZ", camOffsetZ);

	// Get camera offset from input arguments
	camOffset(0) = camOffsetX;
	camOffset(1) = camOffsetY;
	camOffset(2) = camOffsetZ;

	ROS_INFO_STREAM("camOffset = " << camOffsetX << "," << camOffsetY << "," << camOffsetZ);

	// QUATERNION TESTING
	// boost::qvm::quat<double> q12 = {1,3,4,3};
	// boost::qvm::quat<double> q23 = {4,3.9,-1,-3};

	// boost::qvm::quat<double> q34 = boost::qvm::operator*(q12, q23);

	// // Access x,y,z, scalar components of quaternion
	// ROS_INFO_STREAM("quat: = " << boost::qvm::X(q34) << "," << boost::qvm::Y(q34) << "," << boost::qvm::Z(q34) << "," << boost::qvm::S(q34));



	// Define subscriber line to listen to pose from ORBSLAM2
	ros::Subscriber sub_ORBSLAM2_pose = traj_nh.subscribe("/orb_slam2_rgbd/pose", 10, pose_ORBSLAM2_Callback);

	// Define second subscriber for obtaining the tf from the arm
	ros::Subscriber sub_arm_pose = traj_nh.subscribe("/tf", 1, pose_arm_Callback);

    // message_filters::Subscriber<sensor_msgs::Image> image_sub(traj_nh, "/camera/rgb/image_raw", 1);
    // message_filters::Subscriber<tf2_msgs::TFMessage> tf_sub(traj_nh, "/tf", 1);	
    // message_filters::TimeSynchronizer<sensor_msgs::Image, tf2_msgs::TFMessage> sync(image_sub, tf_sub, 10);

    // sync.registerCallback(boost::bind(&pose_arm_Callback, _1, _2));



	// Set up publishing line for path to be sent on
	arm_traj_pub = traj_nh.advertise<nav_msgs::Path>("/tf/trajectory", 1000);
	ORBSLAM2_traj_pub = traj_nh.advertise<nav_msgs::Path>("/orb_slam2_rgbd/trajectory", 1000);

	// Set publishing to run at 60Hz
	ros::Rate loop_rate(15);

	// Run the node until terminated
	while(!g_request_shutdown)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Manual shutdown after ctrl+c
	ros::shutdown();
}