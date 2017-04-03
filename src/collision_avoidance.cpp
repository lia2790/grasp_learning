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

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>

#include <math.h>
#include <stdio.h>
#include <ctime>
#include <time.h>

#include "CollisionAvoidance.h"


using namespace std;
using namespace Eigen;
using namespace KDL;



//////////////////////////////////	BOX

Eigen::VectorXd box(3);
////////////////////////////////////////////////


/////////////////////////////////  CONTACT POINT

Eigen::VectorXd cp(7);
////////////////////////////////////////////////




int main (int argc, char **argv)
{

	ros::init(argc, argv, "collision_avoidance");	// ROS node
	ros::NodeHandle nh;


	string relative_path_file_in;	
	string file_name_in;

	string relative_path_file_out;
	string file_name_out;

	string relative_path_file_hand;
	string file_name_hand;

	nh.param<std::string>("file_name_in", relative_path_file_in, "/box_test/test" );
	nh.param<std::string>("file_name_out", relative_path_file_out, "/box_estimate/" );
	nh.param<std::string>("file_name_hand_point", relative_path_file_hand, "/hand_point/hand_points");



	////////////////////////////////////// load the data_base /////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");


	file_name_in = path + relative_path_file_in;
	ifstream file_in(file_name_in); 


	std::cout << "file: " << file_name_in.c_str() << " is " << (file_in.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_in.is_open())
	return 0;

	
	file_name_hand = path + relative_path_file_hand;
	ifstream file_hand(file_name_hand); 


	std::cout << "file: " << file_name_hand.c_str() << " is " << (file_hand.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_hand.is_open())
	return 0;





	
   	file_name_out = path + relative_path_file_out;


   	ofstream file_output; //output file 
   	std::string name = "box_ca_test";
    file_output.open( file_name_out + name, ofstream::app);

    ofstream file_output_klampt; //output file for KLAMPT
   	std::string name_ = "box_ca_test.csv";
    file_output_klampt.open( file_name_out + name_, ofstream::app);
    ///////////////////////////////////////////////////////////////////////////////////////////////




    //////////////////////////////////////////////////////////////////////////// TAKE HAND POINT

    std::vector<Eigen::VectorXd> hp; //collision point?


    for(std::string line; getline( file_hand, line, '\n' ); )
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    		values_inline.push_back(stod(value));



    	Eigen::VectorXd t(7);

    	double x = values_inline[0];
    	double y = values_inline[1];
    	double z = values_inline[2];

    	double qx = values_inline[3];
    	double qy = values_inline[4];
    	double qz = values_inline[5];
    	double qw = values_inline[6];

    	t(0) = x;
    	t(1) = y;
    	t(2) = z;
    	t(3) = qx;
    	t(4) = qy;
    	t(5) = qz;
    	t(6) = qw;

    	hp.push_back(t);
    }

  ////////////////////////////////////////////////////////////////////////////// /////////////////



  ////////////////////////////////////////////////////////////////////////////// TAKE GRASP POINT


    for(std::string line; getline( file_in, line, '\n' ); )
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    		values_inline.push_back(stod(value));


    	box(0) = values_inline[0];
    	box(1) = values_inline[1];
    	box(2) = values_inline[2];

    	cp(0) = values_inline[3];
    	cp(1) = values_inline[4];
    	cp(2) = values_inline[5];
    	cp(3) = values_inline[6];
    	cp(4) = values_inline[7];
    	cp(5) = values_inline[8];
    	cp(6) = values_inline[9];


    	if(CollisionAvoidance(box, cp, hp))
    	{
    		file_output<<line<<endl;
    		file_output_klampt<<box(0)<<','<<box(1)<<','<<box(2)<<','<<cp(0)<<','<<cp(1)<<','<<cp(2)<<','<<cp(3)<<','<<cp(4)<<','<<cp(5)<<','<<cp(6)<<endl;
    	}
    }

  ////////////////////////////////////////////////////////////////////////////// /////////////////


    cout << "-------------------------------------------------" << endl;
    cout << " You can see your result in this folder : " << (file_name_out + name) << endl;
    cout << "-------------------------------------------------" << endl;


	ros::spinOnce();
	return 0;
}