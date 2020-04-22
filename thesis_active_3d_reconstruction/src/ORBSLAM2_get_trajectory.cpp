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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

// File to save trajectory to - is located in .ros folder
string save_path = "trajectory.bag";

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

nav_msgs::Path path; // Define path as global variable for simplicity

// Have publisher as global variable
ros::Publisher traj_pub; 

// Debug check
int n = 0;

void mySigIntHandler(int sig){
	// Save the trajectory to a bag
	rosbag::Bag bag;
	bag.open(save_path, rosbag::bagmode::Write);

	bag.write("/orb_slam2_rgbd/trajectory", ros::Time::now(), path);
	bag.close();

	g_request_shutdown = 1;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	geometry_msgs::PoseStamped pose;

	path.header = msg->header;

	pose.header = msg->header;
	pose.pose = msg->pose;

	// Debug: Print header of msg
	// ROS_INFO_STREAM("\nReceived pose: " << msg->header);

	// path.header = msg->header;
	path.poses.push_back(pose); // Add new pose to path array

	// Debug - print path - does not work
	// ROS_INFO_STREAM("\nPath: " << path.poses[0].pose);
	// n = n + 1;

	traj_pub.publish(path);
}

int main(int argc, char **argv)
{
	// Initialise ROS, and override SIGINT handler (what happens with ctrl+c) with my own code
	ros::init(argc, argv, "ORBSLAM2_get_trajectory", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);

	ros::NodeHandle traj_nh;

	// Define subscriber line to listen to pose
	ros::Subscriber sub = traj_nh.subscribe("/orb_slam2_rgbd/pose", 1000, poseCallback);

	// Set up publishing line for path to be sent on
	traj_pub = traj_nh.advertise<nav_msgs::Path>("/orb_slam2_rgbd/trajectory", 1000);

	// Set publishing to run at 60Hz
	ros::Rate loop_rate(60);

	// Run the node until terminated
	while(!g_request_shutdown)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Manual shutdown after ctrl+c
	ros::shutdown();
}