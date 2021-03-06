// Active 3D Reconstruction Using a UR5e Robotic Arm

/* Author: Andrew Cao */

/* For Motion Planning API */

#include <iostream>
#include <string>
#include <sstream>
//#include <ros/ros.h>

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



// From moveit_planning_api_tutorial

	// const std::string node_name = "motion_planning_tutorial";
	// ros::init(argc, argv, node_name);
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	// ros::NodeHandle node_handle("~");

// Initialise functions
void setSafetyPlanes(void);

// From move_group tutorial
int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// BEGIN_TUTORIAL
	//
	// Setup
	// ^^^^^
	//
	// MoveIt operates on sets of joints called "planning groups" and stores them in an object called
	// the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
	// are used interchangably.
	static const std::string PLANNING_GROUP = "manipulator"; // Plan based on the end effector/manipulator - see ur5e.srdf for possible planning groups

	// The :move_group_interface:`MoveGroupInterface` class can be easily
	// setup using just the name of the planning group you would like to control and plan for.
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to add and remove collision objects in our "virtual world" scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// Raw pointers are frequently used to refer to the planning group for improved performance.
	const robot_state::JointModelGroup* joint_model_group =
	  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// // Visualization (if using RViz to control)
	// // ^^^^^^^^^^^^^
	// //
	// // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	// namespace rvt = rviz_visual_tools;
	// moveit_visual_tools::MoveItVisualTools visual_tools("ur5e_link0");
	// visual_tools.deleteAllMarkers();

	// // Remote control is an introspection tool that allows users to step through a high level script
	// // via buttons and keyboard shortcuts in RViz
	// visual_tools.loadRemoteControl();

	// // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	// Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	// text_pose.translation().z() = 1.75;
	// visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	// // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	// visual_tools.trigger();

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	// We can get a list of all the groups in the robot:
	ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
	        std::ostream_iterator<std::string>(std::cout, ", "));

	// Start the demo
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the
	// end-effector.
	geometry_msgs::Pose initPose;

	/***************************/
	/*** Adding Safety Planes***/
	/***************************/

	/*** Base Safety Plane - Defining the table  ***/
	// Define a collision object ROS message for the safety plane base.
	moveit_msgs::CollisionObject safetyPlaneBase;
	safetyPlaneBase.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	safetyPlaneBase.id = "zPlane";

	// Define a plane to add to the world - this one is z = 0.12.
	shape_msgs::Plane zPlane;

	// Plane coeffs are in the form {a,b,c,d} where ax + by + cz + d = 0
	zPlane.coef = {0,0,1,-0.12};

	safetyPlaneBase.planes.push_back(zPlane);

	// Express the plane's pose in quaternion form - see https://answers.ros.org/question/9772/quaternions-orientation-representation/

	// Will use the same pose for all planes
	geometry_msgs::Pose planePose;
	planePose.position.x = 0;
	planePose.position.y = 0;
	planePose.position.z = 0;
	planePose.orientation.w = 1; // This specifies a rotation of 0

	// Add pose of zPlane to safetyPlaneBase
	safetyPlaneBase.plane_poses.push_back(planePose);

	safetyPlaneBase.operation = safetyPlaneBase.ADD;

	std::vector<moveit_msgs::CollisionObject> allSafetyPlanes; // Create a vector of objects from the CollisionObject class
	allSafetyPlanes.push_back(safetyPlaneBase); // Add safetyPlaneBase item to the vector


	/*** Positive x safety plane - to left of table ***/

/*	moveit_msgs::CollisionObject xPosSafetyPlane;
	xPosSafetyPlane.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	xPosSafetyPlane.id = "xPosPlane";

	// Define a plane to add to the world - this one is x = 0.45.
	shape_msgs::Plane xPosPlane;
	xPosPlane.coef = {1,0,0,-0.45};

	xPosSafetyPlane.planes.push_back(xPosPlane);

	// Add pose of zPlane to xPosSafetyPlane
	xPosSafetyPlane.plane_poses.push_back(planePose);

	xPosSafetyPlane.operation = xPosSafetyPlane.ADD;

	allSafetyPlanes.push_back(xPosSafetyPlane);


	/*** Negative x safety plane - to right of table ***/

/*	moveit_msgs::CollisionObject xNegSafetyPlane;
	xNegSafetyPlane.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	xNegSafetyPlane.id = "xNegPlane";

	// Define a plane to add to the world - this one is x = -0.45
	shape_msgs::Plane xNegPlane;
	xNegPlane.coef = {1,0,0,0.45};

	xNegSafetyPlane.planes.push_back(xNegPlane);

	// Add pose to plane
	xNegSafetyPlane.plane_poses.push_back(planePose);

	xNegSafetyPlane.operation = xNegSafetyPlane.ADD;

	allSafetyPlanes.push_back(xNegSafetyPlane);

	// /*** Positive y safety plane - in front of robot - not required as tool does not reach past table ***/

	// moveit_msgs::CollisionObject yPosSafetyPlane;
	// yPosSafetyPlane.header.frame_id = move_group.getPlanningFrame();

	// // The id of the object is used to identify it.
	// yPosSafetyPlane.id = "yPosPlane";

	// // Define a plane to add to the world - this one is y = -0.3.
	// shape_msgs::Plane yPosPlane;
	// yPosPlane.coef = {1,0,0,0.3};

	// yPosSafetyPlane.planes.push_back(yPosPlane);

	// // Add pose to plane
	// yPosSafetyPlane.plane_poses.push_back(planePose);

	// yPosSafetyPlane.operation = yPosSafetyPlane.ADD;

	// allSafetyPlanes.push_back(yPosSafetyPlane);

	/*** Negative y safety plane - in front of robot - not required as tool does not reach past table***/

/*	moveit_msgs::CollisionObject yNegSafetyPlane;
	yNegSafetyPlane.header.frame_id = move_group.getPlanningFrame();

	// The id of the object is used to identify it.
	yNegSafetyPlane.id = "yNegPlane";

	// Define a plane to add to the world - this one is y = -0.3
	shape_msgs::Plane yNegPlane;
	yNegPlane.coef = {1,0,0,0.3};

	yNegSafetyPlane.planes.push_back(yNegPlane);

	// Add pose to plane
	yNegSafetyPlane.plane_poses.push_back(planePose);

	yNegSafetyPlane.operation = yNegSafetyPlane.ADD;

	allSafetyPlanes.push_back(yNegSafetyPlane);

*/






	// Add all the safety planes into the world
	ROS_INFO_NAMED("tutorial", "Adding Planes into the world");
	planning_scene_interface.addCollisionObjects(allSafetyPlanes);
		  // Wait for MoveGroup to recieve and process the collision object message
//			  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

	  // // Now when we plan a trajectory it will avoid the obstacle
	  // move_group.setStartState(*move_group.getCurrentState());
	  // geometry_msgs::Pose another_pose;
	  // another_pose.orientation.w = 1.0;
	  // another_pose.position.x = 0.4;
	  // another_pose.position.y = -0.4;
	  // another_pose.position.z = 0.9;
	  // move_group.setPoseTarget(another_pose);

	  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	  // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

	  // // Visualize the plan in RViz
	  // visual_tools.deleteAllMarkers();
	  // visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
	  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	  // visual_tools.trigger();
	  // visual_tools.prompt("next step");

	  // // Now, let's attach the collision object to the robot.
	  // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
	  // move_group.attachObject(safetyPlaneBase.id);

	  // // Show text in RViz of status
	  // visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
	  // visual_tools.trigger();

	  // /* Wait for MoveGroup to recieve and process the attached collision object message */
	  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
	  //                     "robot");

	  // // Now, let's detach the collision object from the robot.
	  // ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
	  // move_group.detachObject(safetyPlaneBase.id);

	  // // Show text in RViz of status
	  // visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
	  // visual_tools.trigger();

	  // /* Wait for MoveGroup to recieve and process the attached collision object message */
	  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
	  //                     "robot");

			  



//	while(1) 
//	{	
		/**** Manual Pose input ****/
	// 	// Variables for storing user input positions
	// float wOrient, xPos, yPos, zPos;
	// std::string stdinString;

		// std::cout << "Enter w orientation: ";
		// std::getline(std::cin,stdinString);
		// std::stringstream(stdinString) >> wOrient;

		// std::cout << "Enter x position: ";
		// std::getline(std::cin,stdinString);
		// std::stringstream(stdinString) >> xPos;

		// std::cout << "Enter y position: ";
		// std::getline(std::cin,stdinString);
		// std::stringstream(stdinString) >> yPos;

		// std::cout << "Enter z position: ";
		// std::getline(std::cin,stdinString);
		// std::stringstream(stdinString) >> zPos;

		// target_pose1.orientation.w = wOrient; // Default 1.0
		// target_pose1.position.x = xPos; // Default 0.3
		// target_pose1.position.y = yPos; // Default 0.5
		// target_pose1.position.z = zPos; // Default 0.3

/** For calibration, hard code a series of poses to travel to - poses also saved on  **/
	float defaultPoseArray[6][7] = {
		{0.383210233685, 0.840890260435, 0.291758403438, 1, 0, 0, 0},
		{0.639339566719, 0.647879142596, 0.373294188842, -0.152975082739, -0.0194413713915, 0.567152000147, 0.809048370537},
		{0.483075273206, 0.757975476777, 0.474561326773, -0.288465799612, 0.0693793698806, 0.473858820744, 0.829115072528},
		{0.237569153749, 0.648627735827, 0.285944255968, -0.0960261939557, 0.0569150628906, 0.259405094058, 0.959295909961},
		{0.0936515835503, 0.769269012194, 0.260153125664, -0.0946328787739, 0.037545617378, 0.129118210009, 0.986389087893},
		{-0.00371977438008, 0.579038915082, 0.245150115465, -0.0326014701386, 0.0515643015861, 0.0592193768509, 0.996379110757}
	};

	for (int poseN = 0; poseN < 1; poseN++) { // Set poseN < 1 for debugging; default <7

		// initPose.position.x = defaultPoseArray[poseN][0];
		// initPose.position.y = defaultPoseArray[poseN][1];
		// initPose.position.z = defaultPoseArray[poseN][2];

		// initPose.orientation.x = defaultPoseArray[poseN][3];
		// initPose.orientation.y = defaultPoseArray[poseN][4];
		// initPose.orientation.z = defaultPoseArray[poseN][5];
		// initPose.orientation.w = defaultPoseArray[poseN][6];

		// Quaternion pose from /tf does not match inputs into moveit
		// From /tf into moveit:
		// x -> -w
		// y -> y
		// z -> x
		// w -> z
		// initPose.orientation.x = defaultPoseArray[poseN][5];
		// initPose.orientation.y = defaultPoseArray[poseN][4];
		// initPose.orientation.z = defaultPoseArray[poseN][6];
		// initPose.orientation.w = -defaultPoseArray[poseN][3];

		// // Overrides
		// initPose.position.x = -0.5;
		// initPose.position.y = -0.5;
		// initPose.position.z = -0.5;
		// initPose.orientation.x = 0;
		// initPose.orientation.y = 0;
		// initPose.orientation.z = 0;
		// initPose.orientation.w = 0.99;

		using namespace std;
		// Manual Input
		cout << "Position x: ";
		cin >> initPose.position.x;
		cout << "Position y: ";
		cin >> initPose.position.y;
		cout << "Position z: ";
		cin >> initPose.position.z;
		cout << "Orient x: ";
		cin >> initPose.orientation.x;
		cout << "Orient y: ";
		cin >> initPose.orientation.y;
		cout << "Orient z: ";
		cin >> initPose.orientation.z;
		cout << "Orient w: ";
		cin >> initPose.orientation.w;


		move_group.setPoseTarget(initPose);

		// Now, we call the planner to compute the plan and visualize it.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

		// Visualizing plans
		// ^^^^^^^^^^^^^^^^^
		// We can also visualize the plan as a line with markers in RViz.
		// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
		// visual_tools.publishAxisLabeled(target_pose1, "pose1");
		// visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
		// visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		// visual_tools.trigger();


		if(success) { // Only move on successful path plan
			std::cout << "Successful path plan. Moving to new position...\n";
			move_group.move();
		}
		else {
			std::cout << "Error setting path! Exiting...\n";
			break;
		}
	}



		

//	}

	//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	// Moving to a pose goal
	// ^^^^^^^^^^^^^^^^^^^^^
	//
	// Moving to a pose goal is similar to the step above
	// except we now use the move() function. Note that
	// the pose goal we had set earlier is still active
	// and so the robot will try to move to that goal. We will
	// not use that function in this tutorial since it is
	// a blocking function and requires a controller to be active
	// and report success on execution of a trajectory.

	/* Uncomment below line when working with a real robot */

			  // // Planning to a joint-space goal
			  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			  // //
			  // // Let's set a joint space goal and move towards it.  This will replace the
			  // // pose target we set above.
			  // //
			  // // To start, we'll create an pointer that references the current robot's state.
			  // // RobotState is the object that contains all the current position/velocity/acceleration data.
			  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
			  // //
			  // // Next get the current set of joint values for the group.
			  // std::vector<double> joint_group_positions;
			  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

			  // // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
			  // joint_group_positions[0] = -1.0;  // radians
			  // move_group.setJointValueTarget(joint_group_positions);

			  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			  // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

			  // // Visualize the plan in RViz
			  // visual_tools.deleteAllMarkers();
			  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
			  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
			  // visual_tools.trigger();
			  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

			  // // Planning with Path Constraints
			  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			  // //
			  // // Path constraints can easily be specified for a link on the robot.
			  // // Let's specify a path constraint and a pose goal for our group.
			  // // First define the path constraint.
			  // moveit_msgs::OrientationConstraint ocm;
			  // ocm.link_name = "panda_link7";
			  // ocm.header.frame_id = "panda_link0";
			  // ocm.orientation.w = 1.0;
			  // ocm.absolute_x_axis_tolerance = 0.1;
			  // ocm.absolute_y_axis_tolerance = 0.1;
			  // ocm.absolute_z_axis_tolerance = 0.1;
			  // ocm.weight = 1.0;

			  // // Now, set it as the path constraint for the group.
			  // moveit_msgs::Constraints test_constraints;
			  // test_constraints.orientation_constraints.push_back(ocm);
			  // move_group.setPathConstraints(test_constraints);

			  // // We will reuse the old goal that we had and plan to it.
			  // // Note that this will only work if the current state already
			  // // satisfies the path constraints. So, we need to set the start
			  // // state to a new pose.
			  // robot_state::RobotState start_state(*move_group.getCurrentState());
			  // geometry_msgs::Pose start_pose2;
			  // start_pose2.orientation.w = 1.0;
			  // start_pose2.position.x = 0.55;
			  // start_pose2.position.y = -0.05;
			  // start_pose2.position.z = 0.8;
			  // start_state.setFromIK(joint_model_group, start_pose2);
			  // move_group.setStartState(start_state);

			  // // Now we will plan to the earlier pose target from the new
			  // // start state that we have just created.
			  // move_group.setPoseTarget(target_pose1);

			  // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
			  // // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
			  // move_group.setPlanningTime(10.0);

			  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
			  // ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

			  // // Visualize the plan in RViz
			  // visual_tools.deleteAllMarkers();
			  // visual_tools.publishAxisLabeled(start_pose2, "start");
			  // visual_tools.publishAxisLabeled(target_pose1, "goal");
			  // visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
			  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
			  // visual_tools.trigger();
			  // visual_tools.prompt("next step");

			  // // When done with the path constraint be sure to clear it.
			  // move_group.clearPathConstraints();

			  // // Cartesian Paths
			  // // ^^^^^^^^^^^^^^^
			  // // You can plan a Cartesian path directly by specifying a list of waypoints
			  // // for the end-effector to go through. Note that we are starting
			  // // from the new start state above.  The initial pose (start state) does not
			  // // need to be added to the waypoint list but adding it can help with visualizations
			  // std::vector<geometry_msgs::Pose> waypoints;
			  // waypoints.push_back(start_pose2);

			  // geometry_msgs::Pose target_pose3 = start_pose2;

			  // target_pose3.position.z -= 0.2;
			  // waypoints.push_back(target_pose3);  // down

			  // target_pose3.position.y -= 0.2;
			  // waypoints.push_back(target_pose3);  // right

			  // target_pose3.position.z += 0.2;
			  // target_pose3.position.y += 0.2;
			  // target_pose3.position.x -= 0.2;
			  // waypoints.push_back(target_pose3);  // up and left

			  // // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
			  // // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
			  // // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
			  // move_group.setMaxVelocityScalingFactor(0.1);

			  // // We want the Cartesian path to be interpolated at a resolution of 1 cm
			  // // which is why we will specify 0.01 as the max step in Cartesian
			  // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
			  // // Warning - disabling the jump threshold while operating real hardware can cause
			  // // large unpredictable motions of redundant joints and could be a safety issue
			  // moveit_msgs::RobotTrajectory trajectory;
			  // const double jump_threshold = 0.0;
			  // const double eef_step = 0.01;
			  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

			  // // Visualize the plan in RViz
			  // visual_tools.deleteAllMarkers();
			  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
			  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
			  // for (std::size_t i = 0; i < waypoints.size(); ++i)
			  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
			  // visual_tools.trigger();
			  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");



			  // // Now, let's remove the collision object from the world.
			  // ROS_INFO_NAMED("tutorial", "Remove the object from the world");
			  // std::vector<std::string> object_ids;
			  // object_ids.push_back(safetyPlaneBase.id);
			  // planning_scene_interface.removeCollisionObjects(object_ids);

			  // // Show text in RViz of status
			  // visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
			  // visual_tools.trigger();

			  // /* Wait for MoveGroup to recieve and process the attached collision object message */
			  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

			  // // END_TUTORIAL


	const std::string end_msg = "\nUR5e Movement Test Complete. Exiting...\n";
	std::cout << end_msg;

	ros::shutdown();
	return 0;
}

void setSafetyPlanes(void) { // Would contain initialisation of safety planes here, however have to deal with scope issues with planning_scene_interface
}