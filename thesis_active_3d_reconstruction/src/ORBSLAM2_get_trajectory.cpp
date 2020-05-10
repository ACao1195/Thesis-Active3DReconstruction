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
#include <Eigen/Geometry>

// Use Boost QVM library for Quaternion calculations
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <boost/qvm/quat_access.hpp>
#include <boost/qvm/mat.hpp>
#include <boost/qvm/mat_operations.hpp>
#include <boost/qvm/mat_access.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/vec_operations.hpp>
#include <boost/qvm/vec_access.hpp>

// Could use all.hpp to include all QVM headers but not recommended
// #include <boost/qvm/all.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_msgs/TFMessage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace boost::qvm;

// Rotate by this much
	// TODO Improvement: Make this a launch arg, allow rotation in other directions
double theta = -0.47; // = 26.92 deg

// vec<float,3> camOffset(0.035, 0.0003, 0.053); // x,y,z camera offset from end effector
vec<float,3> camOffset = {0, 0, 0};

// Quaternion to store the initial translation and orientation of the camera rotated by initial camera position
quat<double> armTransOffset;
quat<double> armOrientOffset;

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

// Marker to represent model and base of arm
visualization_msgs::MarkerArray markerArray;
visualization_msgs::Marker marker;

int firstTF = 0;

int n_ORBSLAM2_pose = 1;
int n_arm_pose = 1;

// Debug check
int n = 0;

// Override on exiting program
void mySigIntHandler(int sig){
	// Save the trajectory to a bag - saves to /.ros/trajectory.bag
	rosbag::Bag bag;
	bag.open(save_path, rosbag::bagmode::Write);

	bag.write("/orb_slam2_rgbd/trajectory", ros::Time::now(), path_ORBSLAM2);
	bag.write("/tf/trajectory", ros::Time::now(), path_arm);
	bag.write("/visual_marker", ros::Time::now(), markerArray);
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
		
	// Extract x,y,z pose, place into quaternion
	boost::qvm::quat<double> ORBSLAM2TransQuat = {0, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
	// Extract orientation
	quat<double> ORBSLAM2OrientQuat = {msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z};

	// Pose = Pose of arm + qv(q^-1), q = initial rotation of arm, v = ORBSLAM2 vector
	quat<double> finalTransQuat = armTransOffset + operator*(operator*(armOrientOffset, ORBSLAM2TransQuat), conjugate(armOrientOffset));

	// Rotation = q3 * q2 * q1, q3 = initial pose of arm, q2 = ORBSLAM2 pose of arm, q1 = initial pose in ORBSLAM2 (0,0,0,1) so q3 * q2
	quat<double> finalOrientQuat = operator*(armOrientOffset,ORBSLAM2OrientQuat);

	// Assign to new pose
	pose.pose.orientation.x = X(finalOrientQuat);
	pose.pose.orientation.y = Y(finalOrientQuat);
	pose.pose.orientation.z = Z(finalOrientQuat);
	pose.pose.orientation.w = S(finalOrientQuat);

	pose.pose.position.x = X(finalTransQuat);
	pose.pose.position.y = Y(finalTransQuat);
	pose.pose.position.z = Z(finalTransQuat);

//	ROS_INFO_STREAM("ORBSLAM2 raw position: " << xPosInit << " y: " << yPosInit << " z: " << zPosInit);



	/*/////////////////
	Manual Rotation
	////////////////////

	// Need to rotate the trajectory so that it is flat with the z-axis
	xPosInit = msg->pose.position.x;
	yPosInit = msg->pose.position.y;
	zPosInit = msg->pose.position.z;

	double cosTheta = cos(theta);
	double sinTheta = sin(theta);

	// Rotate about y-axis and shift to position of arm
	// Flip x and z as they are upside down, and offset by starting position of tf
	pose.pose.position.x = xOffset - (xPosInit * cosTheta + zPosInit * sinTheta);
	pose.pose.position.y = yOffset + (yPosInit);
	pose.pose.position.z = zOffset - (zPosInit * cosTheta - xPosInit * sinTheta);

//	ROS_INFO_STREAM("ORBSLAM2 Pose.pose.position.y = " << pose.pose.position.y);

	using namespace boost::qvm;

	// Apply 180 degree rotation about y-axis
	quat<double> ORBSLAMquat = {msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z};

	// Rotation by 180 deg in y-axis (account for skew correction)
	quat<double> rotQuat = roty_quat(3.14159f + theta);

	// Rotate ORBSLAMquat by rotQuat
	quat<double> finalOrientQuat = boost::qvm::operator*(rotQuat, ORBSLAMquat);

	pose.pose.orientation.x = boost::qvm::X(finalOrientQuat);
	pose.pose.orientation.y = boost::qvm::Y(finalOrientQuat);
	pose.pose.orientation.z = boost::qvm::Z(finalOrientQuat);
	pose.pose.orientation.w = boost::qvm::S(finalOrientQuat);

	*/

	// Caclulate transform from ORBSLAM2 frame of reference (Origin at 0,0,0, rotation 0,0,0,1) to camera frame of reference

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
	if(single_tf.transform.translation.z == 0 && firstTF == 0){
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

		// Place orientation into quaternion
		boost::qvm::quat<double> armQuat = {single_tf.transform.rotation.w, single_tf.transform.rotation.x, single_tf.transform.rotation.y, single_tf.transform.rotation.z};

		// Convert camera offset to quaaternion with real part 0
		quat<double> camOffsetQuat = {0, A<0>(camOffset), A<1>(camOffset), A<2>(camOffset)};

		// Rotate about z-axis 45 deg
		quat<double> rotQuat = boost::qvm::rotz_quat(0.7854f);

		// Calculate pose rotation by p' = qp(q^-1) where q = armQuat and p = camOffsetQuat;

		// Rotate into end effector frame of reference first
		quat<double> camOffsetRotated =  boost::qvm::operator*(operator*(armQuat, camOffsetQuat), conjugate(armQuat));

			// But have to remember camera is tilted by 45 deg relative to axis of end effector - possible that we need to rotate by 45 deg around z-axis of end effector
		// Rotate about z-axis 45 degrees to reach camera angle
//		quat<double> camOrientOffsetFinal = operator*(operator*(rotQuat, camOffsetRotated), conjugate(rotQuat));

		// If 45 degree rotation is already accounted for:
		quat<double> camOrientOffsetFinal = camOffsetRotated;

		// Calculating each part separately
		//quat<double> camOffsetTemp = boost::qvm::operator*(armQuat, camOffsetQuat);
		// quat<double> camOffsetConj = conjugate(camOffsetQuat);
		// quat<double> camOffsetRotated = boost::qvm::operator*(camOffsetTemp, camOffsetConj);

//		camOffsetQuat = conjugate(armQuat);

//		ROS_INFO_STREAM("\nBoost values: s= " << S(camOffsetRotated) << "\nx= " << X(camOffsetRotated) << "\ny= "<< Y(camOffsetRotated) << "\nz= "<< Z(camOffsetRotated));

		// Eigen::Vector3f quatXYZ(single_tf.transform.rotation.x, single_tf.transform.rotation.y, single_tf.transform.rotation.z);
		// float quatW = single_tf.transform.rotation.w;

		// Calculate offset of camera, rotated according to the end effector
		// Rotated vector v' = 2 * dot(u,v) * u + (s^2 - dot(u,u)) * v + 2s (u x v) - where u = [q.x,q.y,q.z]
		// Eigen::Vector3f camOffsetRotatedEigen = 2.0f * (camOffset.dot(quatXYZ)) * quatXYZ 
		// 			+ (quatW * quatW - quatXYZ.dot(quatXYZ)) * camOffset 
		// 			+ 2.0f * quatW * quatXYZ.cross(camOffset);

		// ROS_INFO_STREAM("\ncamOffsetRotated values: " << camOffsetRotatedEigen);

		// Have to reassign translation/position as they are different formats
		pose.pose.position.x = single_tf.transform.translation.x + boost::qvm::X(camOrientOffsetFinal);
		pose.pose.position.y = single_tf.transform.translation.y + Y(camOrientOffsetFinal);
		pose.pose.position.z = single_tf.transform.translation.z + Z(camOrientOffsetFinal);

//		ROS_INFO_STREAM("camOffsetRotated= x:" << X(camOffsetRotated) << ", y:" << Y(camOffsetRotated) << ", z:" << Z(camOffsetRotated));

		// DEBUG: Override value
//		rotQuat = rotz_quat(0.0f);

		// Rotate about 45 degrees about the z-axis
		quat<double> finalOrientQuat = boost::qvm::operator*(armQuat, rotQuat);

		// Remember qvm::quat is in form {w, x, y, z}
		pose.pose.orientation.x = boost::qvm::X(finalOrientQuat);
		pose.pose.orientation.y = Y(finalOrientQuat);
		pose.pose.orientation.z = Z(finalOrientQuat);
		pose.pose.orientation.w = S(finalOrientQuat);

		// Get offsets if it is the first TF message
		if(firstTF == 0) {
			firstTF = 1;
			armTransOffset = {0, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
			armOrientOffset = finalOrientQuat; // Use the camera pose rather than the end effector pose

			// Print starting offsets
			ROS_INFO_STREAM("\n Offsets are: \n x:"<< X(armTransOffset) << "\n y:" << Y(armTransOffset) << "\n z:" << Z(armTransOffset));
		}

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

	// Get camera offset from input arguments, store into vector
	boost::qvm::A<0>(camOffset) = camOffsetX;
	A<1>(camOffset) = camOffsetY;
	A<2>(camOffset) = camOffsetZ;

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

	// Set up a marker to represent the target
	ros::Publisher markerPub = traj_nh.advertise<visualization_msgs::MarkerArray>("/visual_marker", 0);

	// Define marker for model stand-in
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.4;
	marker.pose.position.y = 0.05;
	marker.pose.position.z = 0.1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	markerArray.markers.push_back(marker);

	// Define marker for base of arm
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0.15;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.3;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.6; // Colour grey
	marker.color.g = 0.6;
	marker.color.b = 0.6;

	markerArray.markers.push_back(marker);

	// Set publishing to run at 60Hz
	ros::Rate loop_rate(15);

	// Run the node until terminated
	while(!g_request_shutdown)
	{
		ros::spinOnce();
		markerPub.publish(markerArray);

		loop_rate.sleep();
	}

	// Manual shutdown after ctrl+c
	ros::shutdown();
}