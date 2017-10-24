 /*Author: Emanuele Sansebastiano */

// libs
#include <ros/ros.h>
#include <moveit_user_support/moveit_support.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "script_test_ms");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace bsc = basic_side_classes;
	namespace bsf = basic_side_functions;
	namespace gsf = geometry_side_functions;
	namespace mbf = moveit_basics_functions;
	namespace obf = moveit_object_functions;

	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	bsf::countdown_sec(3);
	//real program init


	ros::shutdown();
	return 0;
}
