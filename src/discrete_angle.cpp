
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




using namespace std;
using namespace Eigen;
using namespace KDL;



int main (int argc, char **argv)
{

	ros::init(argc, argv, "collision_avoidance");	// ROS node
	ros::NodeHandle nh;


	string relative_path_file_in;	
	string file_name_in;
	string relative_path_file_out;
	string file_name_out;


	nh.param<std::string>("file_name_in", relative_path_file_in, "" );
	nh.param<std::string>("file_name_out", relative_path_file_out, "" );



	////////////////////////////////////// load the data_base /////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");


	file_name_in = path + relative_path_file_in;
	ifstream file_in(file_name_in); 


	std::cout << "file: " << file_name_in.c_str() << " is " << (file_in.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_in.is_open())
	return 0;

	
  	file_name_out = path + relative_path_file_out;


  	ofstream f_output; //output file 
	std::string name = "box_discr_angle.csv";
  	f_output.open( file_name_out + name, ofstream::app);


  	////////////////////////////////////////////////////////////////////////////
	///////////////////// take a first pose ////////////////////////////////////	
  	std::string line; 
  	getline( file_in, line, '\n' );
	
	std::vector<double> values_inline;
    std::istringstream iss_line(line);	
    for(std::string value; getline(iss_line, value, ',' ); )
    	values_inline.push_back(stod(value));
    //////////////////////////////////////////////////////////////////////////////////


   double x_box = values_inline[0];
   double y_box = values_inline[1];
   double z_box = values_inline[2];

   double x = values_inline[3];
   double y = values_inline[4];
   double z = values_inline[5];

   double qx = values_inline[6];
   double qy = values_inline[7];
   double qz = values_inline[8];
   double qw = values_inline[9];


   	KDL::Vector t_point(x,y,z);
    KDL::Rotation R_point= Rotation::Quaternion(qx,qy,qz,qw);

    KDL::Frame T_point(R_point,t_point);

    KDL::Vector t(x,y,z);
    KDL::Vector t_app(x,y,z);

    for(int i = 0 ; i < 360; i++)
    {

    	R_point.DoRotZ(i);
    

    	double qx_ = 0;
   		double qy_ = 0;
   		double qz_ = 0;
   		double qw_ = 0;

   		R_point.GetQuaternion(qx_,qy_,qz_,qw_);


    	f_output << x_box << ',' << y_box << ',' << z_box << ',' << t.x() << ',' << t.y() << ',' << t.z() << ',' << qx_ << ',' << qy_ << ',' << qz_ << ',' << qw_ << endl;

    }


}



