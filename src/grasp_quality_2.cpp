/*

Software License Agreement (BSD License)

Copyright (c) 2016--, Liana Bertoni (liana.bertoni@gmail.com)
  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder(s) nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
Contact GitHub API Training Shop Blog About
*/


#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>



#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>


#include <cmath>
#include <ctime>
#include <time.h>


#include "pseudo_inverse.h"
//#include "quality.h"




using namespace std;
using namespace Eigen;
using namespace KDL;






// take it from yaml file
int n_rows;
int n_cols;
int quality_index;
string file_name;
string relative_path_file;
string frame_name_finger[5];
string frame_name_root;
//std::string root_name = "right_hand_softhand_base";
//std::string root_name = "right_hand_palm_link";
//std::string root_name = "world";
int n_fing;




int main (int argc, char **argv)
{

	ros::init(argc, argv, "Grasp_quality");	// ROS node
	ros::NodeHandle nh;


 	
	nh.param<int>("n_rows_file",n_rows,108);
	nh.param<int>("n_cols_file",n_cols,86);
	nh.param<int>("quality_index",quality_index,0);
	nh.param<std::string>("file_name", relative_path_file, "/db/box_db_2.csv" );
	nh.param<std::string>("frame_name_thumb", frame_name_finger[0], "right_hand_thumb_distal_link");
	nh.param<std::string>("frame_name_index", frame_name_finger[1], "right_hand_index_distal_link");
	nh.param<std::string>("frame_name_middle", frame_name_finger[2], "right_hand_middle_distal_link");
	nh.param<std::string>("frame_name_ring", frame_name_finger[3], "right_hand_ring_distal_link");
	nh.param<std::string>("frame_name_little", frame_name_finger[4], "right_hand_little_distal_link");
	nh.param<std::string>("frame_name_root", frame_name_root, "world");
	nh.param<int>("number_of_fingers", n_fing, 5);



	







	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");
	file_name = path + relative_path_file;
	ifstream file(file_name); 

	std::cout << "file: " << file_name.c_str() << " is " << (file.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file.is_open())
	return 0;


	
	///////////////////// laod the urdf model //////////////////////////////////
	KDL::Tree hand_tree;
	KDL::Chain chains_hand_finger[n_fing];
	KDL::Jacobian hand_jacob[n_fing];
	KDL::JntArray q_finger[n_fing];
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver[n_fing];



	std::string robot_desc_string;
	nh.param("robot_description", robot_desc_string, string());  // robot description is the name in the launch file 
	if (!kdl_parser::treeFromString(robot_desc_string, hand_tree))
		{ ROS_ERROR("Failed to construct kdl tree"); return false;}
  

	for(int i=0; i < n_fing; i++) // get chain for each fingers
	{	
		hand_tree.getChain(frame_name_root, frame_name_finger[i], chains_hand_finger[i]);      
		q_finger[i] = JntArray(chains_hand_finger[i].getNrOfJoints());
		jnt_to_jac_solver[i].reset(new KDL::ChainJntToJacSolver(chains_hand_finger[i]));
		q_finger[i].resize(chains_hand_finger[i].getNrOfJoints());
		hand_jacob[i].resize(chains_hand_finger[i].getNrOfJoints());
	}






	///////////////////// get values from file for each line //////////////////////////////////






	int i = 0;
	int j = 0;
	std::vector<double> values_inline;


	for(std::string line; getline( file, line, '\n' ); ) // for each line
	{

    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ',' ); )
    		values_inline.push_back(stod(value));
    		
    	

    
    	
	} // for each line













	cout << " YEAH ENJOY " << endl;
	cout << "   fine   " << endl;


	ros::spin();
	return 0;
}