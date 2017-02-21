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

#include <math.h>
#include <stdio.h>
#include <ctime>
#include <time.h>

#include "PGR_3.h"
#include "quality.h"
#include "normal_component_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;




int main (int argc, char **argv)
{

	ros::init(argc, argv, "Grasp_quality");	// ROS node
	ros::NodeHandle nh;



	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	nh.param<int>("quality_index",quality_index,0);
	nh.param<int>("number_of_fingers", n_fingers, 5);
	nh.param<std::string>("frame_name_root", frame_name_root, "world");
	nh.param<std::string>("frame_name_thumb", frame_name_finger[0], "right_hand_index_distal_link" );
	nh.param<std::string>("frame_name_index", frame_name_finger[1], "right_hand_little_distal_link" );
	nh.param<std::string>("frame_name_middle", frame_name_finger[2], "right_hand_middle_distal_link");
	nh.param<std::string>("frame_name_ring", frame_name_finger[3], "right_hand_ring_distal_link");
	nh.param<std::string>("frame_name_little", frame_name_finger[4], "right_hand_thumb_distal_link");
	nh.param<std::string>("file_name", relative_path_file, "/db/box_db_2.csv" );
	nh.param<double>("joint_stiffness",joint_stiffness,0);
	nh.param<double>("contact_stiffness",contact_stiffness,0);
	nh.param<double>("mu", mu, 0.03);
	nh.param<double>("f_i_max", f_i_max, 1);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////











	cout << endl;
	cout << "Count rows : " << count_line << endl;
	cout << "quality_index : " << quality_index << endl;
	
	cout << " YEAH ENJOY " << endl;
	cout << "   fine   " << endl;

	ros::spinOnce();
	return 0;

}