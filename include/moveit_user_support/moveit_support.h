/* Author: 	Emanuele Sansebastiano
 * Date:	August 2017
 *
 * Desc:	Library to encapsulate some user helping function based on MoveIt! pkg
 *
 */

#ifndef MOVEIT_USER_SUPPORT_MOVEIT_SUPPORT_H
#define MOVEIT_USER_SUPPORT_MOVEIT_SUPPORT_H

// ROS
#include <ros/ros.h>
////#include <sys/time.h>

// C++
#include <fstream>
////#include <tf/transform_listener.h>

// Sansebastiano's lib
#include <side_pkg/side_func.h>

//side_func.h's libs
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/math/constants/constants.hpp>
////#include <geometry_msgs/PointStamped.h>
#include <tf_conversions/tf_kdl.h>

// Moveit! libraries
////#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Baxter libraries
#include <baxter_core_msgs/EndpointState.h>
////#include <baxter_core_msgs/SEAJointState.h>


//Macro Concatenation
#define MAC_SUM(A, B) A # B

//////////////////////////////////////////////////////////////////////////////////////
// VALUES MODIFIABLE BY THE USER \\

// Common define values
#define	stdAttempts4booleanFunc		5
#define exit_function_time			0.5 //[sec]
//Moveit! values
#define att_exit					5 //maximum # moveit planner attempts
#define time_exit					12.0 //maximum time value moveit planner can use
//The following define structures could change according to the robot
#define	generic_str					""
//Baxter default values
//DO NOT MODIFY THESE VALUES, except if it is strictly necessary!
#define right_def					"right"
#define left_def					"left"
#define gripper_def					"_gripper"
#define	std_head_frame				"/world"
#define arm_group_name				"_arm"
#define both_arms_group_name		"both_arms"
#define tf_tree_starter_name		"torso"
//baxter topic defintion
#define joint_state					"/robot/joint_states"
#define base_robot_part				"/robot/limb/"
#define end_point					"/endpoint_state"
//baxter default values
#define baxter_def_joint_toll		0.0001		//[deg]
#define baxter_def_orient_toll		0.001		//[deg]
#define baxter_def_pos_toll 		0.0001		//[m]
#define baxter_attempts				1
#define baxter_max_factor			1.0
//scene values Empty boxes
#define std_thickness				0.02		//[m]


//brief: Function defining the joint names in your Robot | user can change the joints names
//       In case you receive errors or it is not complete, use the function 'obj_functions::getJointNames()' to check it
std::vector<std::string> joint_names()
{
	//Just modify the following line if strictly necessary
	//Check '#define tf_tree_starter_name' definition
	const std::string joints_names [] = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
	std::vector<std::string> j_n;
	j_n.resize(sizeof(joints_names)/sizeof(*joints_names));
	for(int i = 0; i < j_n.size(); i++)
		j_n[i] = joints_names[i];

	return j_n;
}

//brief: Function defining the link names in your Robot | user can change the link names
//       In case you receive errors or it is not complete, use the function 'obj_functions::getLinkNames()' to check it
std::vector<std::string> link_names()
{
	//Just modify the following line if strictly necessary
	const std::string links_names [] = {"_upper_shoulder", "_lower_shoulder", "_upper_elbow", "_lower_elbow", "_upper_forearm", "_lower_forearm", "_hand", "_gripper_base", "_gripper"};
	std::vector<std::string> l_n;
	l_n.resize(sizeof(links_names)/sizeof(*links_names));
	for(int i = 0; i < l_n.size(); i++)
		l_n[i] = links_names[i];

	return l_n;
}

//////////////////////////////////////////////////////////////////////////////////////

/*HOW TO USE THIS LIBRARY
   At the beginning of you program you must insert the following lines:
 * 	ros::init(argc, argv, "your_node_name");
 *	ros::NodeHandle node_handle("~");
 *	ros::AsyncSpinner spinner(1);
 *  spinner.start();
 */


namespace moveit_basics_functions
{
  //Comment: Other constraints could be set like 'PositionConstraints', but they are useless for this project.
  //         Check moveit_msgs/msg/Constraints.msg file to know more about them.
  //brief: Function to define a orientation constraint parameter from quaternions
  //       there is a function to know which is the planning frame "group.getPlanningFrame()" default = "/world"
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Quaternion orientation, std::string link_name,
  		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = std_head_frame);
  //brief: Function to define a orientation constraint parameter from RPY in degree
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY_orientation, std::string link_name,
   		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = std_head_frame);

  //brief: Function to return the list of the available planners
  //       OMPL planner names might change (change them in the .cpp function)
  std::vector<std::string> getOmplPlannerList();

  //brief: Function to list all the possible basic solid shapes
  //       solid names might change (change them in the .cpp function)
  std::vector<std::string> get_possible_solid_shapes();

  //brief: Function to generate a collision object (SPHERE)
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj,
    		  geometry_msgs::Vector3 position, double radius, std::string solid_type = "SPHERE", std::string header_frame_id = std_head_frame);

  //brief: Function to generate a collision object (BOX) -- Using RPY in degree
  //       there is a function to know which is the planning frame "group.getPlanningFrame()" default = "/world"
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);
  //brief: Function to generate a collision object (BOX) -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);

  //brief: Function to generate a collision object (Cylinder or Cone) -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);
  //brief: Function to generate a collision object (Cylinder or Cone) -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);

  //brief: Function to generate a collision object (BOX) -- Using RPY in degree
  //       define the position of the solid by the the centre of the solid base instead of the centre of the whole solid
  //       in "collision_obj_generator" the solid position is defined by the centre of it. No rotation along the z_solid_axis
  //       WARNING: DO NOT ROTATE ON X AND Y TOGETHER, it shift the base a bit
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Vector3 orientation,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Quaternion quat,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);
  //brief: same as the previous one, but for CYLINDER and CONE.
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Vector3 orientation,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Quaternion quat,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);

  //brief: Function to generate an empty box opened, it can be rotated only on the z_axis
  //       the variable "dimension" corresponds to the empty space inside the box,
  //       the out-side dimension are the inside dimension plus the thickness of the box material.
  //       The bottom of the box has a fixed layer of 0.01 [m] | open the cpp file if you want to change it.
  std::vector<moveit_msgs::CollisionObject> CollisionEmptyBox(std::string id_emptyBox, geometry_msgs::Vector3 position, geometry_msgs::Vector3 dimension, double z_rotation = 0.0, double thickness = std_thickness);
  //Comment on the object generator: Using different id_name is really important!
  //                      			 Two object with the same id_name cannot exist! (overwritten)

  //brief: Function to read from a file a set of collision object
  std::vector<moveit_msgs::CollisionObject> readCollisionObj(std::string file_directory);

  //brief: Function to get the pose of the arm frames from the /tf
  //       The selected frames derives by the function 'link_names()', check it in case of errors
  //std::vector<geometry_msgs::PoseStamped> TF_arm_point_pose(std::string right_left);

  //brief: Functions to get the Endpoint State, Pose, Twist, Wrench
  //       at the beginning of your program you must have the following lines:
  //         ros::init(argc, argv, "your_node_name");
  //         ros::NodeHandle node_handle("~");
  baxter_core_msgs::EndpointState getEndPointStateFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Pose getEePoseFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Twist getEeTwistFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Wrench getEeWrenchFromTopic(std::string right_left, ros::NodeHandle &nh);

  //brief: Functions to get the arm joints position, velocity, effort
  //       at the beginning of your program you must have the following lines:
  //         ros::init(argc, argv, "ur_node_name");
  //         ros::NodeHandle node_handle("~");
  sensor_msgs::JointState getBothArmJointValFromTopic(std::string right_left, ros::NodeHandle &nh);
  //One arm only!
  std::vector<double> getOneArmJointPositionFromTopic(std::string right_left, ros::NodeHandle &nh);
  std::vector<double> getOneArmJointVelocityFromTopic(std::string right_left, ros::NodeHandle &nh);
  std::vector<double> getOneArmJointEffortFromTopic(std::string right_left, ros::NodeHandle &nh);
  //Both arms (left joints before - right joints after in the vector)
  std::vector<double> getBothArmJointPositionFromTopic(ros::NodeHandle &nh);
  std::vector<double> getBothArmJointVelocityFromTopic(ros::NodeHandle &nh);
  std::vector<double> getBothArmJointEffortFromTopic(ros::NodeHandle &nh);

// End namespace "moveit_basics_functions"
}

namespace moveit_object_functions
{
  ////// ROBOT FUNCTIONS //////
  // The object must be instantiated in the executable file using:
  // moveit::planning_interface::MoveGroup obj_name(std::string part_name)
  // part_name could be: "right_arm", "left_arm", or "both_arms"

  //brief: This function returns the list of the possible group names
  std::vector<std::string> GroupNameAvailable();

  //brief: Function to get the link names
  std::vector<std::string> getLinkNames(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to get the joint names
  std::vector<std::string> getJointNames(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to define the Work Space Box
  //       This function has to be used when the planning frame is the world ("/world").
  bool setWorkSpaceBox(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Vector3 min_XYZ, geometry_msgs::Vector3 max_XYZ);

  //brief: Function to change the planner ID among the ones available
  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner);

  //brief: Function to change the planning time
  bool setPlanningTime(moveit::planning_interface::MoveGroup& obj, double new_time);

  //brief: Function to set the number of planning attempts - default = 1
  bool setPlanningAttempts(moveit::planning_interface::MoveGroup& obj, unsigned int attempts = baxter_attempts);

  //brief: Function to set the max velocity factor in (0.0 ; 1.0] - default = 1.0
  bool setMaxVelocityFactor(moveit::planning_interface::MoveGroup& obj, double val = baxter_max_factor);

  //brief: Function to set the max acceleration factor in (0.0 ; 1.0] - default = 1.0
  bool setMaxAccelerationFactor(moveit::planning_interface::MoveGroup& obj, double val = baxter_max_factor);

  //brief: Function to set the pose of the end-effector
  //       !Do not forget there is a function to generate a Pose msg (basic_side_functions::makePose)
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj,
		  geometry_msgs::Pose pose, std::string left_right = generic_str);

  //brief: Function to impose the joint value target directly
  //       Change "r_l_both" just in case both arms are controlled, but you want to move just one of them
  //       Do not forget to cluster the joint position with the function: 'basic_side_functions::vector_two_cluster'.
  //       The values in the vector must go from the shoulder to the wrist.
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj,
		  std::vector<double> vector, std::string r_l_single = generic_str);

  //brief: Function to set the joint tolerance - data given in degree
  //       If you want to set the default tolerance do not put any tolerance value
  bool setJointTolerance(moveit::planning_interface::MoveGroup& obj, double toll = baxter_def_joint_toll);

  //brief: Function to set the end-effector goal orientation tolerance - data given in degree
  //       If you want to set the default tolerance do not put any tolerance value
  bool setEeOrientationTolerance(moveit::planning_interface::MoveGroup& obj, double toll = baxter_def_orient_toll);

  //brief: Function to set the end-effector goal position tolerance - data given in meter
  //       If you want to set the default tolerance do not put any tolerance value
  bool setEePositionTolerance(moveit::planning_interface::MoveGroup& obj, double toll = baxter_def_pos_toll);

  //brief: Function to sum all the constraints already defined in one big data to pass to the function "setConstraint"
  //       Orientation check the function "orient_constr_definition"
  //	   You must generate a data "moveit_msgs::Constraints data_name" in your main
  void Constraints_definition(moveit_msgs::Constraints &constraints, moveit_msgs::OrientationConstraint or_con);
  //       Position (not very usefull)
  void Constraints_definition(moveit_msgs::Constraints &constraints, moveit_msgs::PositionConstraint pos_con);

  //brief: Function to set constraints
  bool setConstraint(moveit::planning_interface::MoveGroup& obj, moveit_msgs::Constraints constraint);

  //brief: Function to clear all the constraints
  bool clearConstraints(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to plan and execute safely:
  //       if the path plan is not found it return a false msg and the robot do not move
  bool plan_execute_safely(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to plan and execute safely giving back the time of every action:
  //       it return 2 values: fist planning time in seconds; second executing time; negative values in case of not path found
  std::vector<double> plan_execute_safely_t(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to plan and execute safely, increasing the planning time gradually until "#define time_exit"
  //       if "restore_initial == true" the previous value is restored in the end
  bool plan_execute_safely_time_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial = true);

  //brief: Function to plan and execute safely, increasing the planning attempts gradually until "#define att_exit"
  //       if "restore_initial == true" the default value (1) is restored in the end
  bool plan_execute_safely_attempt_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial = true);

  //brief: Function to plan and move directly [not safe]
  void move_direct(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to stop any trajectory motion, if one is running
  void motion_stop(moveit::planning_interface::MoveGroup& obj);


  ////// ENVIRONMENT FUNCTIONS //////
  // The object must be instantiated in the executable file using:
  // moveit::planning_interface::PlanningSceneInterface obj_name

  //brief: Function to define color RGBA fast (RGB is RGB, while S is the transparence)
  std_msgs::ColorRGBA color_RGBA(float r, float g, float b, float a = 1.0);

  //brief: Function to get the color code by the name
  std_msgs::ColorRGBA get_color_code(std::string color_name);

  //brief: Function to add an object to the scene
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj, std::string color_name = "GREEN");
  //brief: The same but used for more than one object clustered in a vector
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, std::vector<moveit_msgs::CollisionObject> &coll_obj, std::string color_name = "GREEN");

  //brief: Function to remove an object to the scene
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj);
  //brief: The same but used for more than one object clustered in a vector
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, std::vector<moveit_msgs::CollisionObject> &coll_obj);

  //brief: Function to attach a specific object to a moveit group
  //       obj_id = obj_in_the_scene.id
  bool attachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, std::string link_id = generic_str, double time2wait = std_time);

  //brief: Function to detach a specific object to a moveit group
  //       obj_id = obj_in_the_scene.id
  bool detachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait = std_time);

  //brief: Function to get all the objects published in the scene by Moveit!
  std::vector<moveit_msgs::CollisionObject> GetMoveitSceneObjects(moveit::planning_interface::PlanningSceneInterface &interface);

// End namespace "moveit_object_functions"
}


#endif /* MOVEIT_SIDE_PKG_SIDE_FUNCTIONS_H */

/*
 * Main library for MoveIt!: move_group.h
 * Directory of move_group.h: ~/moveit/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface
 *
 * Object Constructor: MoveGroup
 * Object Destructor: ~MoveGroup
 *
 * Interesting functions included in this object definition:
 * - getName **
 * - getNameTarget **
 * - getRobotName **
 * - getNodeHandle **
 * - getJointName //
 * - getLinkName //
 * - getActiveJoints **
 * - getJoints **
 * - getDefaultPlannerId **
 * - getCurrentJointValues //
 * - getCurrentState  **
 * - getCurrentPose //
 * - getCurrentRPY //
 *
 * - setPlannerId //
 * - setPlanningTime //
 * - setNumPlanningAttempts //
 * - setMaxVelocityScalingFactor //
 * - setMaxAccelerationScalingFactor //
 * - setWorkSpace //
 * - setStartState
 * - setStartStateToCurrentState
 * - setSupportSurfaceName **
 * - setJointValueTarget
 * - setApproximateJointValueTarget
 * - setPositionTarget
 * - setRPYTarget
 * - setOrientationTarget
 * - setPoseTarget //
 * - setPathConstraints //
 * - setGoalJointTolerance //
 * - setGoalPositionTolerance //
 * - setGoalOrientationTolerance //
 *
 * - clearPoseTarget **
 * - clearPathConstraints //
 *
 * - attachObject //
 * - detachObject //
 *
 * - computeCartesianPath
 * - Move //
 * - Plan //
 * - Execute //
 * - asyncMove **
 * - asyncExecute **
 * - Stop //
 * - Pick **
 * - Place **
 *
 */


