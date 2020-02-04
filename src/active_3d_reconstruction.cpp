// Active 3D Reconstruction Using a UR5e Robotic Arm

/* Author: Andrew Cao */

/* For Motion Planning API */

#include <ros/ros.h>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h> // This include fails	
// //#include <pluginlib/class_loader.h>

// #include <boost/scoped_ptr.hpp>

/****************************/
/* For move_group interface */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv) {
	std::string testString("Hello world\n");
	std::cout << testString;

// From moveit_planning_api_tutorial

	// const std::string node_name = "motion_planning_tutorial";
	// ros::init(argc, argv, node_name);
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	// ros::NodeHandle node_handle("~");

	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::cout << testString;


	return 0;
}