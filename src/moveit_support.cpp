/* Author: 	Emanuele Sansebastiano
 * Date:	August 2017
 *
 * Desc:	Library to encapsulate some user helping function based on MoveIt! pkg
 *
 */

// this pkg
#include <moveit_user_support/moveit_support.h>


namespace moveit_basics_functions
{
  //Other namespaces:
  namespace bsc = basic_side_classes;
  namespace bsf = basic_side_functions;
  namespace gsf = geometry_side_functions;

  //Global variable for this namespace
  baxter_core_msgs::EndpointState gl_end_point_state;
  sensor_msgs::JointState gl_joints_state;

  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Quaternion orientation, std::string link_name, float toll_x, float toll_y, float toll_z, float weight, std::string header_frame_id)
  {
	  moveit_msgs::OrientationConstraint orient_constr;
	  orient_constr.orientation = orientation;
	  orient_constr.link_name = link_name;
	  orient_constr.absolute_x_axis_tolerance = toll_x;
	  orient_constr.absolute_y_axis_tolerance = toll_y;
	  orient_constr.absolute_z_axis_tolerance = toll_z;
	  orient_constr.weight = weight;
	  orient_constr.header.frame_id = header_frame_id;

	  return orient_constr;
  }
  //RPY_orientation in degree
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY_orientation, std::string link_name, float toll_x, float toll_y, float toll_z, float weight, std::string header_frame_id)
  {
      moveit_msgs::OrientationConstraint orient_constr;
  	  orient_constr.orientation = gsf::RPY2Quat(RPY_orientation, false);
  	  orient_constr.link_name = link_name;
  	  orient_constr.absolute_x_axis_tolerance = toll_x;
  	  orient_constr.absolute_y_axis_tolerance = toll_y;
  	  orient_constr.absolute_z_axis_tolerance = toll_z;
  	  orient_constr.weight = weight;
  	  orient_constr.header.frame_id = header_frame_id;

  	  return orient_constr;
  }

  std::vector<std::string> getOmplPlannerList(void)
  {
	  std::vector<std::string> planner_list;
	  planner_list.resize(8);
	  std::string base = "ConfigDefault";
	  planner_list[0] = "BKPIECE" + base;
	  planner_list[1] = "EST" + base;
	  planner_list[2] = "KPIECE" + base;
	  planner_list[3] = "LBKPIECE" + base;
	  planner_list[4] = "RRTConnect" + base;
	  planner_list[5] = "RRTStar" + base;
	  planner_list[6] = "RRT" + base;
	  planner_list[7] = "SBL" + base;

	  return planner_list;
  }

  std::vector<std::string> get_possible_solid_shapes(void)
  {
	  std::vector<std::string> solids;
	  solids.resize(5);
	  solids[0] = "BOX";
	  solids[1] = "SPHERE";
	  solids[2] = "CYLINDER";
	  solids[3] = "CONE";
	  solids[4] = "EMPTY_BOX";

	  return solids;
  }

  //Collision object generator: SPHERE
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, double radius, std::string solid_type, std::string header_frame_id)
  {
	  //Collision object definition
	  moveit_msgs::CollisionObject collision_object;
	  collision_object.id = id_collision_obj;
	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
	  collision_object.header.frame_id = header_frame_id;

	  //solid definition
	  shape_msgs::SolidPrimitive output_solid;
	  output_solid.type = shape_msgs::SolidPrimitive::SPHERE;
	  output_solid.dimensions.resize(1);
	  output_solid.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;

	  //solid pose definition
	  geometry_msgs::Pose solid_pose;
	  //the orientation is not important
	  solid_pose.orientation.w = 1.0;
	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

	  collision_object.primitives.push_back(output_solid);
	  collision_object.primitive_poses.push_back(solid_pose);

	  return collision_object;
  }

  //Collision object generator: BOX -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;
  	  output_solid.type = shape_msgs::SolidPrimitive::BOX;
  	  output_solid.dimensions.resize(3);
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimension.x;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimension.y;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimension.z;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = gsf::RPY2Quat(orientation, false);
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }
  //Collision object generator: BOX -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;
  	  output_solid.type = shape_msgs::SolidPrimitive::BOX;
  	  output_solid.dimensions.resize(3);
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimension.x;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimension.y;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimension.z;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = quat;
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }

  //Collision object generator: CYLINDER or CONE -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation, double height, double radius, std::string solid_type, std::string header_frame_id)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;

  	  output_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  	  if(solid_type == "CONE")
  		  output_solid.type = shape_msgs::SolidPrimitive::CONE;
  	  else if(solid_type != "CONE" && solid_type != "CYLINDER")
  		  std::cout << "Warning: A default cylinder has been created, but the solid type inserted was not valid. '" << solid_type << "' is not valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important

  	  solid_pose.orientation = gsf::RPY2Quat(orientation, false);

  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }
  //Collision object generator: CYLINDER or CONE -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat, double height, double radius, std::string solid_type, std::string header_frame_id)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;

  	  output_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  	  if(solid_type == "CONE")
  		  output_solid.type = shape_msgs::SolidPrimitive::CONE;
  	  else if(solid_type != "CONE" && solid_type != "CYLINDER")
  		  std::cout << "Warning: A default cylinder has been created, but the solid type inserted was not valid. '" << solid_type << "' is not valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = quat;
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }

  //Collision object generator: BOX -- Using RPY in degree and position defined in the center of the base
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
  {
	  geometry_msgs::Vector3 z_moveit_position;

	  std::vector <std::vector <double> > temp_matrix;
	  std::vector <std::vector <double> > rot_matrix = gsf::RPY2rotMatrix(orientation);
	  std::vector <std::vector <double> > trans_matrix = gsf::translation_matrix(positionCentreBase);
	  std::vector <std::vector <double> > point_matrix = gsf::translation_matrix(gsf::makeVector3(0.0, 0.0, dimension.z/2));

	  bsf::matrix2DPROD(trans_matrix, rot_matrix, temp_matrix);
	  bsf::matrix2DPROD(temp_matrix, point_matrix, temp_matrix);

	  z_moveit_position = gsf::transMatrix2XYZ(temp_matrix);

	  moveit_msgs::CollisionObject collision_object = collision_obj_generator(id_collision_obj, z_moveit_position, orientation, dimension, solid_type, header_frame_id);

   	  return collision_object;
  }
  //Collision object generator: BOX -- Using quaternion and position defined in the center of the base
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Quaternion quat, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
  {
	  geometry_msgs::Vector3 orientation = gsf::Quat2RPY(quat);
	  geometry_msgs::Vector3 z_moveit_position;

	  std::vector <std::vector <double> > temp_matrix;
	  std::vector <std::vector <double> > rot_matrix = gsf::RPY2rotMatrix(orientation);
	  std::vector <std::vector <double> > trans_matrix = gsf::translation_matrix(positionCentreBase);
	  std::vector <std::vector <double> > point_matrix = gsf::translation_matrix(gsf::makeVector3(0.0, 0.0, dimension.z/2));

	  bsf::matrix2DPROD(trans_matrix, rot_matrix, temp_matrix);
	  bsf::matrix2DPROD(temp_matrix, point_matrix, temp_matrix);

	  z_moveit_position = gsf::transMatrix2XYZ(temp_matrix);

	  moveit_msgs::CollisionObject collision_object = collision_obj_generator(id_collision_obj, z_moveit_position, quat, dimension, solid_type, header_frame_id);

   	  return collision_object;
  }

  //Collision object generator: CYLINDER or CONE -- Using RPY in degree and position defined in the center of the base
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Vector3 orientation, double height, double radius, std::string solid_type, std::string header_frame_id)
  {
	  geometry_msgs::Vector3 z_moveit_position;

	  std::vector <std::vector <double> > temp_matrix;
	  std::vector <std::vector <double> > rot_matrix = gsf::RPY2rotMatrix(orientation);
	  std::vector <std::vector <double> > trans_matrix = gsf::translation_matrix(positionCentreBase);
	  std::vector <std::vector <double> > point_matrix = gsf::translation_matrix(gsf::makeVector3(0.0, 0.0, height/2));

	  bsf::matrix2DPROD(trans_matrix, rot_matrix, temp_matrix);
	  bsf::matrix2DPROD(temp_matrix, point_matrix, temp_matrix);

	  z_moveit_position = gsf::transMatrix2XYZ(temp_matrix);

	  moveit_msgs::CollisionObject collision_object = collision_obj_generator(id_collision_obj, z_moveit_position, orientation, height, radius, solid_type, header_frame_id);

   	  return collision_object;
  }
  //Collision object generator: CYLINDER or CONE -- Using quaternion and position defined in the center of the base
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 positionCentreBase, geometry_msgs::Quaternion quat, double height, double radius, std::string solid_type, std::string header_frame_id)
  {
	  geometry_msgs::Vector3 orientation = gsf::Quat2RPY(quat);
	  geometry_msgs::Vector3 z_moveit_position;

	  std::vector <std::vector <double> > temp_matrix;
	  std::vector <std::vector <double> > rot_matrix = gsf::RPY2rotMatrix(orientation);
	  std::vector <std::vector <double> > trans_matrix = gsf::translation_matrix(positionCentreBase);
	  std::vector <std::vector <double> > point_matrix = gsf::translation_matrix(gsf::makeVector3(0.0, 0.0, height/2));

	  bsf::matrix2DPROD(trans_matrix, rot_matrix, temp_matrix);
	  bsf::matrix2DPROD(temp_matrix, point_matrix, temp_matrix);

	  z_moveit_position = gsf::transMatrix2XYZ(temp_matrix);

	  moveit_msgs::CollisionObject collision_object = collision_obj_generator(id_collision_obj, z_moveit_position, quat, height, radius, solid_type, header_frame_id);

   	  return collision_object;
  }

  std::vector<moveit_msgs::CollisionObject> CollisionEmptyBox(std::string id_emptyBox, geometry_msgs::Vector3 position, geometry_msgs::Vector3 dimension, double z_rotation, double thickness)
  {
	  double rot_rad = bsf::deg2rad(z_rotation);
	  //Bottom creation
	  double bottom_thickness = 0.01;
	  geometry_msgs::Vector3 dim;
	  dim.x = dimension.x; dim.y = dimension.y; dim.z = bottom_thickness;
	  geometry_msgs::Vector3 orientation;
	  orientation.x = 0.0; orientation.y = 0.0; orientation.z = z_rotation;
	  geometry_msgs::Vector3 pos = position;
	  std::string part_name;
	  part_name = id_emptyBox + "_bottom";
	  moveit_msgs::CollisionObject bottom = collision_obj_generator_z(part_name, pos, orientation, dim);

	  //side 1 and 3 creation
	  dim.x = dimension.x + 2*thickness; dim.y = thickness; dim.z = dimension.z;
	  pos.x = position.x - ((dimension.y + thickness)/2) * sin(rot_rad);
	  pos.y = position.y + ((dimension.y + thickness)/2) * cos(rot_rad);
	  pos.z = position.z + dimension.z/2;
	  part_name = id_emptyBox + "_side1";
	  moveit_msgs::CollisionObject side1 = collision_obj_generator(part_name, pos, orientation, dim);
	  pos.x = position.x + ((dimension.y + thickness)/2) * sin(rot_rad);
	  pos.y = position.y - ((dimension.y + thickness)/2) * cos(rot_rad);
	  part_name = id_emptyBox + "_side3";
	  moveit_msgs::CollisionObject side3 = collision_obj_generator(part_name, pos, orientation, dim);

	  //side 2 and 4 creation
	  dim.x = thickness; dim.y = dimension.y + 2*thickness; dim.z = dimension.z;
	  pos.x = position.x + ((dimension.x + thickness)/2) * cos(rot_rad);
	  pos.y = position.y + ((dimension.x + thickness)/2) * sin(rot_rad);
	  pos.z = position.z + dimension.z/2;
	  part_name = id_emptyBox + "_side2";
	  moveit_msgs::CollisionObject side2 = collision_obj_generator(part_name, pos, orientation, dim);
	  pos.x = position.x - ((dimension.x + thickness)/2) * cos(rot_rad);
	  pos.y = position.y - ((dimension.x + thickness)/2) * sin(rot_rad);
	  part_name = id_emptyBox + "_side4";
	  moveit_msgs::CollisionObject side4 = collision_obj_generator(part_name, pos, orientation, dim);

	  std::vector<moveit_msgs::CollisionObject> box_vector;
	  box_vector.push_back(bottom);
	  box_vector.push_back(side1);
	  box_vector.push_back(side2);
	  box_vector.push_back(side3);
	  box_vector.push_back(side4);

	  return box_vector;
  }

  std::vector<moveit_msgs::CollisionObject> readCollisionObj(std::string file_directory)
  {
	  std::vector<moveit_msgs::CollisionObject> obj_set;

	  //reading input parameter
	  bsc::UsefulCharString chars;
	  char newline_char = chars.get_newline_char();
	  char tab_char = chars.get_tab_char();
	  char comment_char = chars.get_comment_char();
	  std::string id_str_temp;
	  double radius_temp, height_temp, ori_z_temp, thick_temp;
	  double in3_doub[3];
	  geometry_msgs::Vector3 pos_centre_temp, orientation_temp, dimension_temp;
	  moveit_msgs::CollisionObject obj_temp;
	  std::vector<moveit_msgs::CollisionObject> empty_box_obj_temp;
	  //reading input parameter to check
	  std::vector<std::string> id_objs;

	  //function variable
	  char temp_char;
	  int obj_int;
	  int obj_counter = 0;
	  std::string temp_str;
	  std::vector<std::string> obj_kind = get_possible_solid_shapes();

	  std::ifstream file(file_directory, std::ios::in);
	  if(!file.is_open())
	  {
		  std::cout << "The file cannot be opened to read the scene definition, an empty vector has been returned" << std::endl;
		  perror("CHECK IF IT EXISTS IN THIS DIRECTORY!");
	  }else{
		  file >> temp_char;
		  while(!file.eof())
		  {
			  temp_str.clear(); //reinitializing of temp_str [very importatnt]
			  if(temp_char == comment_char)
			  {	  // read the comment until it is not closed
				  temp_char = comment_char-1; //to de-initialize the variable
				  while(temp_char != comment_char)
					  {file >> temp_char;}
			  }else{
				  // read one word until the tab
				  while(temp_char != tab_char)
				  {	  temp_str += temp_char;
					  temp_char = file.get();
				  }
				  // string check
				  obj_int = -1;
				  for(int i = 0; i < obj_kind.size(); i++)
				  {	  if(temp_str == obj_kind[i])
						  obj_int = i;
				  }
				  if (obj_int < 0)
				  {
					  std::cout << "The obstacle called: " << temp_str << " is not included in the system" << std::endl;
					  std::cout << "This object is going to be skept, but check the file and change the obstacle definition to one of the following:" << std::endl;
					  for(int i = 0; i < obj_kind.size(); i++)
						  std::cout << obj_kind[i] << std::endl;
					  // read the entire line until
					  while(temp_char != newline_char)
					  {	   temp_char = file.get();}
				  }else{

					  file >> id_str_temp;
					  // check object having the same name
					  for(int i = 0; i < id_objs.size(); i++)
					  {
						  if(id_str_temp == id_objs[i])
						  {
							  std::cout << "Warning: the object called '" << id_str_temp << "' already exists" << std::endl;
							  id_str_temp += "a";
							  std::cout << "The new object name is '" << id_str_temp << "', but changing it in the read file is strongly suggested" << std::endl;
						  }
					  }
					  id_objs.push_back(id_str_temp);

					  file >> in3_doub[0] >> in3_doub[1] >> in3_doub[2];
					  pos_centre_temp = gsf::makeVector3(in3_doub[0], in3_doub[1], in3_doub[2]);

					  if(obj_int == 0) //BOX
					  {	  file >> in3_doub[0] >> in3_doub[1] >> in3_doub[2];
						  orientation_temp = gsf::makeVector3(in3_doub[0], in3_doub[1], in3_doub[2]);
						  file >> in3_doub[0] >> in3_doub[1] >> in3_doub[2];
						  dimension_temp = gsf::makeVector3(in3_doub[0], in3_doub[1], in3_doub[2]);

						  //object generation and push_back
						  obj_temp = collision_obj_generator_z(id_str_temp, pos_centre_temp, orientation_temp, dimension_temp);
						  obj_set.push_back(obj_temp);
						  obj_counter++;

					  }else if(obj_int == 1){ //SPHERE
						  file >> radius_temp;

						  //object generation and push_back
						  obj_temp = collision_obj_generator(id_str_temp, pos_centre_temp, radius_temp);
						  obj_set.push_back(obj_temp);
						  obj_counter++;

					  }else if(obj_int > 1 && obj_int < 4){ //CYLINDER or CONE
						  file >> in3_doub[0] >> in3_doub[1] >> in3_doub[2];
						  orientation_temp = gsf::makeVector3(in3_doub[0], in3_doub[1], in3_doub[2]);
						  file >> radius_temp >> height_temp;

						  //object generation and push_back
						  obj_temp = collision_obj_generator_z(id_str_temp, pos_centre_temp, orientation_temp, radius_temp, height_temp, obj_kind[obj_int]);
						  obj_set.push_back(obj_temp);
						  obj_counter++;

					  }else if(obj_int == 4){ //EMPTY_BOX
						  file >> in3_doub[0] >> in3_doub[1] >> in3_doub[2];
						  dimension_temp = gsf::makeVector3(in3_doub[0], in3_doub[1], in3_doub[2]);
						  file >> ori_z_temp >> thick_temp;

						  if(thick_temp <= 0.0)// default thickness
							  thick_temp = std_thickness;

						  //object generation and push_back
						  empty_box_obj_temp = CollisionEmptyBox(id_str_temp, pos_centre_temp, dimension_temp, ori_z_temp, thick_temp);
						  for(int i = 0; i < empty_box_obj_temp.size(); i++)
							  obj_set.push_back(empty_box_obj_temp[i]);
						  obj_counter++;
					  }
				  }
			  }
			  file >> temp_char;
		  }
	  }
	  std::cout << obj_counter << " collision object loaded correctly" << std::endl;

	  return obj_set;
  }

  /*std::vector<geometry_msgs::PoseStamped> TF_arm_point_pose(std::string right_left)
  {
	  tf::TransformListener listener(ros::Duration(10));
	  std::vector<geometry_msgs::PoseStamped> point_poses;
	  geometry_msgs::PoseStamped temp_pose_father;
	  geometry_msgs::PoseStamped temp_pose_child;
	  std::string temp_str;
	  bool done;

	  //torso initialization
	  temp_pose_father.header.frame_id = tf_tree_starter_name;
	  temp_pose_father.header.stamp = ros::Time();
	  temp_pose_father.pose.orientation = gsf::RPY2Quat(gsf::makeVector3(0.0, 0.0, 0.0));

	  if(right_left != right_def && right_left != left_def){
		  std::cout << "Warning: insert correctly from which arm you want to get the current joint values.";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will return an empty message by default." << std::endl;
	  }else{
		  std::vector<std::string> l_n = link_names();
		  point_poses.resize(l_n.size());

		  for(int i = 0; i < l_n.size(); i++)
		  {
			  temp_str = tf_tree_starter_name;
			  temp_pose_father.header.frame_id = right_left + l_n[i];

			  done = false;
			  while(!done)
			  {
				  try{
					  listener.transformPose(temp_str, temp_pose_father, temp_pose_child);
					  //std::cout << temp_pose_father.header.frame_id << " --> " << temp_str << " | " << temp_pose_child.header.frame_id << std::endl;
					  temp_pose_child.header.frame_id =temp_pose_father.header.frame_id;
					  point_poses[i] = temp_pose_child;
					  //preparation for the next iteration
					  temp_pose_father.header.stamp = ros::Time();

					  done = true;

				  }catch(tf::TransformException& ex){
					 //std::cout << "Received an exception trying to transform poses from frame " << temp_pose_father.header.frame_id << " to frame " << temp_str <<": " << ex.what() << std::endl;
				  }
			  }
		  }
	  }

	  return point_poses;
  }*/

  //Callback for the end-effector endpoint state (position, twist, wrench) | global variable
  void callback_Ee(const baxter_core_msgs::EndpointState data){
	  gl_end_point_state = data;
	  //to check if the callback function is running uncomment the following line
	  //std::cout << "XYZ.x: " << data.pose.position.x << std::endl;
  }
  baxter_core_msgs::EndpointState getEndPointStateFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  baxter_core_msgs::EndpointState eps2return;
	  if(right_left != right_def && right_left != left_def){
		  std::cout << "Warning: insert correctly from which gripper you want to get the current information. ";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will return an empty message by default." << std::endl;
	  }else{
		  std::string topic_str = base_robot_part + right_left + end_point;
		  if(bsf::CheckTopicExistence(topic_str)){
			  //initialization of the exit param
			  ros::Subscriber sub = nh.subscribe <baxter_core_msgs::EndpointState>(topic_str, 10, callback_Ee);
			  double curr_time = 0.0; double step_time = std_time;
			  //do not proceed until the topic is not read correctly or the maximum time expired "#define exit_function_time"
			  while (eps2return.pose.position.x == 0.0 && eps2return.pose.position.y == 0.0 && curr_time < exit_function_time)
			  {
				  eps2return = gl_end_point_state;
				  //the robot cannot be in the position (0.0,0.0,z) cuz it is inside the body
				  if(eps2return.pose.position.x == 0.0 && eps2return.pose.position.y == 0.0){
					  curr_time += step_time;
					  bsf::standardSleep(step_time);
					  ros::spinOnce();
				  }
			  }
			  //reinitialization of the global variable
			  baxter_core_msgs::EndpointState default_gl_end;
			  gl_end_point_state = default_gl_end;
		  }else{
			  std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
			  std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
			  std::cout << "This function returned an empty message by default." << std::endl;
		  }
	  }

	  return eps2return;
  }

  geometry_msgs::Pose getEePoseFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
	  return eps2return.pose;
  }
  geometry_msgs::Twist getEeTwistFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
  	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
  	  return eps2return.twist;
  }
  geometry_msgs::Wrench getEeWrenchFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
  	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
  	  return eps2return.wrench;
  }

  //Callback for the joints state (position, velocity, effort) | global variable
  void callback_Joint(sensor_msgs::JointState data){
	  gl_joints_state = data;}
  sensor_msgs::JointState getBothArmJointValFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  sensor_msgs::JointState joints_val2return;
	  if(right_left != right_def && right_left != left_def){
		  std::cout << "Warning: insert correctly from which arm you want to get the current joint values.";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will return an empty message by default." << std::endl;
	  }else{
		  std::string topic_str = joint_state;
		  if(bsf::CheckTopicExistence(topic_str)){
			  //initialization of the exit param
			  double curr_time = 0.0; double step_time = std_time;
			  double control_param = 3000.0; joints_val2return.position.push_back(control_param);
			  //do not proceed until the topic is not read correctly or the maximum time expired "#define exit_function_time"
			  ros::Subscriber sub = nh.subscribe(topic_str, 10, callback_Joint);
			  while (joints_val2return.position[0] == control_param && curr_time < exit_function_time)
			  {
				  joints_val2return = gl_joints_state;
				  curr_time += step_time;
				  //the robot joint rotation value cannot be high as control_param
				  if(joints_val2return.position[0] == control_param){
					  curr_time += step_time;
					  bsf::standardSleep(step_time);
					  ros::spinOnce();
				  }
			  }
			  // restore the original wrong value
			  if(joints_val2return.position[0] == control_param){
				  joints_val2return.position[0] = 0;}

			  //reinitialization of the global variable
			  sensor_msgs::JointState default_gl_joints;
			  gl_joints_state = default_gl_joints;
		  }else{
			  std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing.";
			  std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
			  std::cout << "This function returned an empty message by default." << std::endl;
		  }
	  }

	  return joints_val2return;
  }

  std::vector<double> getOneArmJointPositionFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.position[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }
  std::vector<double> getOneArmJointVelocityFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.velocity[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }
  std::vector<double> getOneArmJointEffortFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.effort[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }

  std::vector<double> getBothArmJointPositionFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointPositionFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointPositionFromTopic(right_def,nh);

	  joints_val2return = bsf::vector_cluster_double(left_temp, right_temp);

	  return joints_val2return;
  }
  std::vector<double> getBothArmJointVelocityFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointVelocityFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointVelocityFromTopic(right_def,nh);

	  joints_val2return = bsf::vector_cluster_double(left_temp, right_temp);

	  return joints_val2return;
  }
  std::vector<double> getBothArmJointEffortFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointEffortFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointEffortFromTopic(right_def,nh);

	  joints_val2return = bsf::vector_cluster_double(left_temp, right_temp);

	  return joints_val2return;
  }

// End namespace "moveit_basics_functions"
}


namespace moveit_object_functions
{
  //Other namespaces:
  namespace bsf = basic_side_functions;
  namespace gsf = geometry_side_functions;
  namespace mbf = moveit_basics_functions;

  ////// ROBOT FUNCTIONS //////
  std::vector<std::string> GroupNameAvailable(void)
  {
	  std::vector<std::string> names;
	  names.resize(3);
	  names[0] = MAC_SUM(left_def, arm_group_name);
	  names[1] = MAC_SUM(left_def, arm_group_name);
	  // the option "both_arms" must be always the last one
	  names[2] = both_arms_group_name;

	  return names;
  }

  std::vector<std::string> getLinkNames(moveit::planning_interface::MoveGroup& obj)
  {
	  std::vector<std::string> LinkNames = obj.getLinkNames();
	  return LinkNames;
  }

  std::vector<std::string> getJointNames(moveit::planning_interface::MoveGroup& obj)
  {
  	  std::vector<std::string> JointNames = obj.getJointNames();
  	  return JointNames;
  }

  //NO effective check
  bool setWorkSpaceBox(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Vector3 min_XYZ, geometry_msgs::Vector3 max_XYZ)
  {
	  if(obj.getPlanningFrame() != "/world")
		  std::cout << "Warning: you are not planning on the '/world' frame. You are planning on the '" << obj.getPlanningFrame() << "'. Be careful!" << std::endl;

	  obj.setWorkspace(min_XYZ.x, min_XYZ.y, min_XYZ.z, max_XYZ.x, max_XYZ.y, max_XYZ.z);

	  return true;
  }

  //NO effective check (Planner existance check)
  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner)
  {
	  bool success = false;
	  std::string obj_group = obj.getName();
	  std::string planner_ID = obj.getDefaultPlannerId(obj_group);
	  std::vector<std::string> planner_list = moveit_basics_functions::getOmplPlannerList();
	  int temp = planner_list.size();
	  for (int i = 0; i < temp; i++){
		  if (planner_list[i] == new_planner){
			  success = true;
		      break;
		  }
	  }
	  if (success){
		  obj.setPlannerId(new_planner);
		  std::cout << "Done correctly: The new planner for the group " << obj_group << " is " << new_planner << "." << std::endl;
      }else{
		  std::cout << "Warning: The planner chosen do not exit in OMPL Default. Call the function 'moveit_basics_functions::getOmplPlannerList() to check the available planners." << std::endl;
	      std::cout << "Warning: The planner for the group " << obj_group << " is still the last one successfully set." << std::endl;
     }

	 return success;
  }

  //YES effective check
  bool setPlanningTime(moveit::planning_interface::MoveGroup& obj, double new_time)
  {
	  ros::AsyncSpinner temp_spinner(1);
	  temp_spinner.start();
	  bool success = false;
	  if (new_time < 0.0){
		  std::cout << "Warning: The planning time cannot be negative! Previous value is maintained." << std::endl;
	  }else{

		  //effective setting check
		  int exit_count = 0;
		  success = false;
		  while (!success && exit_count <= stdAttempts4booleanFunc){
			  obj.setPlanningTime(new_time);
			  bsf::standardSleep();
			  if(obj.getPlanningTime() == new_time){
				  success = true;
				  std::cout << "Done correctly: The new planning time of the group " << obj.getName() << " is " << new_time << "." << std::endl;
			  }else{
				  exit_count++;
			  }
		  }
	  }

	  temp_spinner.stop();
	  return success;
  }

  //NO effective check
  bool setPlanningAttempts(moveit::planning_interface::MoveGroup& obj, unsigned int attempts)
  {
	  obj.setNumPlanningAttempts(attempts);

	  return true;
  }

  //NO effective check
  bool setMaxVelocityFactor(moveit::planning_interface::MoveGroup& obj, double val)
  {
	  if(val > 0.0 && val <= 1.0){
		  obj.setMaxVelocityScalingFactor(val);
		  return true;
	  }else{
		  std::cout << "Warning: the factor must be in the interval (0.0; 1.0]! You inserted " << val << "." <<std::endl;
		  return false;
	  }
  }

  //NO effective check
  bool setMaxAccelerationFactor(moveit::planning_interface::MoveGroup& obj, double val)
  {
	  if(val > 0.0 && val <= 1.0){
		  obj.setMaxAccelerationScalingFactor(val);
		  return true;
	  }else{
		  std::cout << "Warning: the factor must be in the interval (0.0; 1.0]! You inserted " << val << "." <<std::endl;
		  return false;
	  }
  }

  //YES effective check
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Pose pose, std::string left_right)
  {
	  bool success = false;
	  std::vector<std::string> pos_groups = GroupNameAvailable();
	  std::string gripper;

	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != right_def && left_right != left_def)){
		  std::cout << "Warning: insert correctly to which gripper you want to assign to that pose msg. ";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will not return positive value." << std::endl;
		  return false;
	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == right_def || left_right == left_def)){
		  gripper = left_right + gripper_def;
	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
		  gripper = obj.getEndEffectorLink();
	  }

	  //effective setting check
	  int exit_count = 0;
	  while (!success && exit_count <= stdAttempts4booleanFunc){
		  obj.setPoseTarget(pose, gripper);
	  	  bsf::standardSleep();
	  	  geometry_msgs::Pose goal_pose = gsf::PoseStamped2Pose(obj.getPoseTarget(gripper));
	  	  if(gsf::PoseEquivalence_decimal_value(goal_pose, pose, 2)){
	  		  success = true;
	  		  std::cout << "Done correctly: The goal pose of the " << gripper << " of the group " << obj.getName() << " has been set." << std::endl;
	  	  }else{
	  		  exit_count++;
	  	  }
	  }

	  return success;
  }

  //--- effective check
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj, std::vector<double> vector, std::string r_l_single)
  {
	  // All the possible groups
	  std::vector<std::string> pos_groups = GroupNameAvailable();
	  // Current object group name
	  std::string GroupName = obj.getName();
	  // Current group joints names
	  std::vector<std::string> jointsName = obj.getJointNames();
	  // Number possible groups
	  int num_groups = pos_groups.size();
	  // Number of joints
	  int size = jointsName.size();

	  // CHECK AND CONTROL
	  if(GroupName == pos_groups[num_groups-1]){
		  if(r_l_single == right_def || r_l_single == left_def)
			  size = size/2;
	  }
	  if(size != vector.size())
		  return false;

	  // if I want to change the position of just one arm and I am controlling both arms
	  if(GroupName == pos_groups[num_groups-1] && size != jointsName.size()){

		  int count = 0;
		  for(int i = 0; i < jointsName.size(); i++){
			  if(jointsName[i].find(r_l_single) >= 0){
				  obj.setJointValueTarget(jointsName[i], vector[count]);
				  count++;
			  }
		  }
	  // if I want to change the position of all the arms I am controlling
	  }else{
		  obj.setJointValueTarget(vector);
	  }

	  return true;
  }

  //YES effective check
  bool setJointTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = bsf::abs_f(toll);
	  }
	  //degree to radiant conversion
	  double toll_rad = bsf::deg2rad(toll);

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= stdAttempts4booleanFunc)
	  {
		  obj.setGoalJointTolerance(toll_rad);
		  bsf::standardSleep();
		  if(obj.getGoalJointTolerance() == toll_rad){
			  success = true;
			  std::cout << "Done correctly: The new joint tolerance of the group " << obj.getName() << " is " << toll << " degree." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //YES effective check
  bool setEeOrientationTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = bsf::abs_f(toll);
	  }
	  //degree to radiant conversion
	  double toll_rad = bsf::deg2rad(toll);

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= stdAttempts4booleanFunc)
	  {
		  obj.setGoalOrientationTolerance(toll_rad);
		  bsf::standardSleep();
		  if(obj.getGoalOrientationTolerance() == toll_rad){
			  success = true;
			  std::cout << "Done correctly: The new end-effector orientation tolerance of the group " << obj.getName() << " is " << toll << " degree." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //YES effective check
  bool setEePositionTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = bsf::abs_f(toll);
	  }

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= stdAttempts4booleanFunc)
	  {
		  obj.setGoalPositionTolerance(toll);
		  bsf::standardSleep();
		  if(obj.getGoalPositionTolerance() == toll){
			  success = true;
			  std::cout << "Done correctly: The new end-effector position tolerance of the group " << obj.getName() << " is " << toll << " meter." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //No effective check
  void Constraints_definition(moveit_msgs::Constraints &constraints, moveit_msgs::OrientationConstraint or_con)
  {
	  constraints.orientation_constraints.push_back(or_con);
  }
  void Constraints_definition(moveit_msgs::Constraints &constraints, moveit_msgs::PositionConstraint pos_con)
  {
	  constraints.position_constraints.push_back(pos_con);
  }

  //-- effective check
  bool setConstraint(moveit::planning_interface::MoveGroup& obj, moveit_msgs::Constraints constraint)
  {
	  obj.setPathConstraints(constraint);

	  return true;
  }

  //NO effective check
  bool clearConstraints(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.clearPathConstraints();

	  return true;
  }

  //YES effective check
  bool plan_execute_safely(moveit::planning_interface::MoveGroup& obj)
  {
	  ros::AsyncSpinner temp_spinner(1);
	  temp_spinner.start();
	  bool success = false;
	  moveit::planning_interface::MoveGroup::Plan my_plan;
	  success = obj.plan(my_plan);
	  if(success)
		  obj.execute(my_plan);
	  else
	    std::cout << "Something went wrong during planning!" << std::endl;

	  temp_spinner.stop();
	  return success;
  }

  std::vector<double> plan_execute_safely_t(moveit::planning_interface::MoveGroup& obj)
  {
	  ros::AsyncSpinner temp_spinner(1);
	  temp_spinner.start();
	  std::vector<double> time_used;
	  long int temp_time1; long int temp_time2;
	  time_used.resize(2);
	  bool success = false;
	  moveit::planning_interface::MoveGroup::Plan my_plan;

	  temp_time1 = bsf::getCurrentTime();
	  success = obj.plan(my_plan);
	  temp_time2 = bsf::getCurrentTime();

	  if(success){
		  time_used[0] = (temp_time2 - temp_time1); time_used[0] /= 1000;

		  temp_time1 = bsf::getCurrentTime();
		  obj.execute(my_plan);
		  temp_time2 = bsf::getCurrentTime();
		  time_used[1] = (temp_time2 - temp_time1); time_used[1] /= 1000;

		  //uncomment the following line to check the execution
		  //std::cout << "The time spent for planning and the executing is: " << time_used[0] << " + " << time_used[1] << std::endl;
	  }
	  else{
		  time_used[0] = -1.0; time_used[1] = -1.0;
		  std::cout << "Something went wrong during planning!" << std::endl;
	  }
	  temp_spinner.stop();
	  return time_used;
  }


  //YES effective check
  bool plan_execute_safely_time_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial)
  {
	  bool success = false;
	  double curr_time = obj.getPlanningTime();
	  double initial_curr_time = curr_time;
	  while(!success && curr_time <= time_exit)
	  {
		  success = plan_execute_safely(obj);
		  if (!success){
			  curr_time += 2.0;
			  setPlanningTime(obj, curr_time);
		  }
	  }
	  //restore the previous value
	  if(initial_curr_time != curr_time && restore_initial){
		  std::cout << "Current planning time has been set back to: " << initial_curr_time << std::endl;
		  setPlanningTime(obj, initial_curr_time);
	  }

	  return success;
  }

  //YES effective check
  bool plan_execute_safely_attempt_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial)
  {
	  bool success = false;
	  unsigned int default_att = 1;
	  unsigned int curr_att = default_att;
	  while(!success && curr_att <= att_exit)
	  {
		  success = plan_execute_safely(obj);
		  if (!success){
			  curr_att += 2;
			  setPlanningAttempts(obj, curr_att);
		  }
	  }
	  //restore the previous the default value
	  if(default_att != curr_att && restore_initial)
		  setPlanningAttempts(obj, default_att);

	  return success;
  }

  //NO effective check
  void move_direct(moveit::planning_interface::MoveGroup& obj)
  {
	  ros::AsyncSpinner temp_spinner(1);
	  temp_spinner.start();
	  obj.move();
	  temp_spinner.stop();
  }

  //NO effective check
  void motion_stop(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.stop();
  }


  ////// ENVIRONMENT FUNCTIONS //////
  std_msgs::ColorRGBA color_RGBA(float r, float g, float b, float a)
  {
	  std_msgs::ColorRGBA output_color;
	  output_color.r = r;
	  output_color.g = g;
	  output_color.b = b;
	  output_color.a = a;

	  return output_color;
  }

  std_msgs::ColorRGBA get_color_code(std::string color_name)
  {
	  std::vector<std_msgs::ColorRGBA> color_list_code;
	  std::vector<std::string> color_list_name;

	  //color list
	  color_list_name.push_back("GREEN");
	  color_list_code.push_back(color_RGBA(0.0,1.0,0.0));
	  color_list_name.push_back("GREEN2");
	  color_list_code.push_back(color_RGBA(0.0,0.5,0.0));
	  color_list_name.push_back("RED");
	  color_list_code.push_back(color_RGBA(1.0,0.0,0.0));
	  color_list_name.push_back("RED2");
	  color_list_code.push_back(color_RGBA(0.5,0.0,0.0));
	  color_list_name.push_back("BLUE");
	  color_list_code.push_back(color_RGBA(0.0,0.0,1.0));
	  color_list_name.push_back("BLUE2");
	  color_list_code.push_back(color_RGBA(0.0,0.0,0.5));
	  color_list_name.push_back("BLACK");
	  color_list_code.push_back(color_RGBA(0.0,0.0,0.0));
	  color_list_name.push_back("WHITE");
	  color_list_code.push_back(color_RGBA(1.0,1.0,1.0));
	  color_list_name.push_back("GRAY");
	  color_list_code.push_back(color_RGBA(0.75,0.75,0.75));
	  color_list_name.push_back("GRAY2");
	  color_list_code.push_back(color_RGBA(0.5,0.5,0.5));
	  color_list_name.push_back("YELLOW");
	  color_list_code.push_back(color_RGBA(1.0,1.0,0.0));
	  color_list_name.push_back("YELLOW2");
	  color_list_code.push_back(color_RGBA(0.5,0.5,0.0));
	  color_list_name.push_back("FUCHSIA");
	  color_list_code.push_back(color_RGBA(1.0,0.0,1.0));
	  color_list_name.push_back("FUCHSIA2");
	  color_list_code.push_back(color_RGBA(0.5,0.0,0.5));
	  color_list_name.push_back("WATER");
	  color_list_code.push_back(color_RGBA(0.0,1.0,1.0));
	  color_list_name.push_back("WATER2");
	  color_list_code.push_back(color_RGBA(0.0,0.5,0.5));
	  //end of the list

	  std_msgs::ColorRGBA output_color;
	  bool match_found;

	  for(int i = 0; i < color_list_name.size(); i++)
		  if(color_name == color_list_name[i])
		  {
			  match_found = true;
			  output_color = color_list_code[i];

		  }else if(i == color_list_name.size()-1 && !match_found){
			  std::cout << "Warning: the color '" << color_name << "' is not implemented, the default color ('" << color_list_name[0] << "') has been set" << std::endl;
			  output_color = color_list_code[0];
			  std::cout << "The implemented colors are:" << std::endl;
			  for(int i = 0; i < color_list_name.size(); i++)
				  std::cout << color_list_name[i] << std::endl;
		  }

	  return output_color;
  }

  //NO effective check
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj, std::string color_name)
  {
	  std_msgs::ColorRGBA color_obj = get_color_code(color_name);
	  coll_obj.operation = coll_obj.ADD;
	  interface.applyCollisionObject(coll_obj, color_obj);

	  return true;
  }
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, std::vector<moveit_msgs::CollisionObject> &coll_obj, std::string color_name)
  {
	  std_msgs::ColorRGBA color_obj = get_color_code(color_name);

/*	  std::vector<moveit_msgs::ObjectColor> vector_color_obj;
	  moveit_msgs::ObjectColor temp_ObjColor;
	  for(int i = 0; i < coll_obj.size(); i++)
	  {
	  	  coll_obj[i].operation = moveit_msgs::CollisionObject::ADD;
		  temp_ObjColor.color = color_obj;
		  temp_ObjColor.id = coll_obj[i].id + "_color";
		  vector_color_obj.push_back(temp_ObjColor);
	  	  std::cout << vector_color_obj[i].id << " " << vector_color_obj[i].color << std::endl;
	  }
	  vector_color_obj.resize(vector_color_obj.size());
	  //addCollisionObjects(coll_obj, vector_color_obj);
	  interface.applyCollisionObjects(coll_obj, vector_color_obj);
*/

	  for(int i = 0; i < coll_obj.size(); i++)
	  {
	   	  coll_obj[i].operation = moveit_msgs::CollisionObject::ADD;
	  	  interface.applyCollisionObject(coll_obj[i], color_obj);
	  }

  	  return true;
  }

  /* MOVEIT MISSING FUNCTION ABOUT addObject
   * Might happen that the functions 'addObject' do not compile.
   * Probably it happens bacause your version of Moveit! does not have the following function:
   *
   * /////////
   * bool PlanningSceneInterface::applyCollisionObject(const moveit_msgs::CollisionObject& collision_object, const std_msgs::ColorRGBA& object_color)
   * {
   *   moveit_msgs::PlanningScene ps;
   *   ps.robot_state.is_diff = true;
   *   ps.is_diff = true;
   *   ps.world.collision_objects.reserve(1);
   *   ps.world.collision_objects.push_back(collision_object);
   *   moveit_msgs::ObjectColor oc;
   *   oc.id = collision_object.id;
   *   oc.color = object_color;
   *   ps.object_colors.push_back(oc);
   *   return applyPlanningScene(ps);
   * }
   *
   * bool PlanningSceneInterface::applyCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects, const std::vector<moveit_msgs::ObjectColor>& object_colors)
   * {
   *   moveit_msgs::PlanningScene ps;
   *   ps.robot_state.is_diff = true;
   *   ps.is_diff = true;
   *   ps.world.collision_objects = collision_objects;
   *   ps.object_colors = object_colors;
   *   return applyPlanningScene(ps);
   * }
   * /////////
   *
   * Copy and paste the previous lines in the library called: 'planning_scene_interface.cpp'
   *
   * and copy and paste the following line in the header file called 'planning_scene_interface.h'
   *
   * ////////
   * bool applyCollisionObject(const moveit_msgs::CollisionObject& collision_object, const std_msgs::ColorRGBA& object_color);
   * bool applyCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects, const std::vector<moveit_msgs::ObjectColor>& object_colors);
   * ////////
   */

  //NO effective check
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj)
  {
	  coll_obj.operation = coll_obj.REMOVE;
	  interface.applyCollisionObject(coll_obj);

	  return true;
  }
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, std::vector<moveit_msgs::CollisionObject> &coll_obj)
    {
	  int size = coll_obj.size();
	  for(int i = 0; i < size; i++)
	  {
	  	  coll_obj[i].operation = moveit_msgs::CollisionObject::REMOVE;
	  	  interface.applyCollisionObject(coll_obj[i]);
	  }

  	  return true;
    }

  //NO effective check (partial check on the link name)
  bool attachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, std::string link_id,  double time2wait)
  {
	  std::vector<std::string> link_names = getLinkNames(group);
	  for(int i = 0; i < link_names.size(); i++)
	  {
		  if(link_names[i] == link_id){
			  std::cout << "The object " << obj_id << " has been attached to the link " << link_id << " of the group " << group.getName() << " correctly." << std::endl;
			  group.attachObject(obj_id, link_id);
			  break;
		  }else if(i == link_names.size() -1 && link_names[i] == generic_str){
			  std::cout << "The object " << obj_id << " has been attached to the default link " << group.getEndEffectorLink() << " of the group " << group.getName() << " correctly." << std::endl;
			  group.attachObject(obj_id);
			  break;
		  }else if(i == link_names.size() -1 && link_names[i] != generic_str){
			  std::cout << "Error: The object " << obj_id << " has been attached to the default link " << group.getEndEffectorLink() << " of the group " << group.getName() << " because the link " << link_id << " does not exist!" << std::endl;
			  std::cout << "Call the function 'obj_function::getLinkNames(" << group.getName() << ") to check the available link names." << std::endl;
			  return false;
		  }
	  }
	  bsf::standardSleep(time2wait);

	  return true;
  }

  //NO effective check
  bool detachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait)
  {
  	  group.detachObject(obj_id);
	  bsf::standardSleep(time2wait);

  	  return true;
  }

  std::vector<moveit_msgs::CollisionObject> GetMoveitSceneObjects(moveit::planning_interface::PlanningSceneInterface &interface)
  {
	  std::vector<moveit_msgs::CollisionObject> obj_list;
	  std::vector<std::string> obj_id = interface.getKnownObjectNames();
	  std::map<std::string, moveit_msgs::CollisionObject > objpres = interface.getObjects();

	  for(int i = 0; i < obj_id.size(); i++)
	  {
		  obj_list.push_back(objpres[obj_id[i]]);
	  }

	  return obj_list;
  }

// End namespace "moveit_object_functions"
}
