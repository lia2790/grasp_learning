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

#include "PGR_5.h"
#include "quality.h"
#include "normal_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;


int main (int argc, char **argv)
{

	ros::init(argc, argv, "grasp_dataset_populate");	// ROS node
	ros::NodeHandle nh;

	string relative_path_file;
	string file_name;
	nh.param<std::string>("file_name", relative_path_file, "/box_db_2/box_db_quality_PGR_matlab" );



	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");
	file_name = path + relative_path_file;
	ifstream file(file_name); 


	double q = 0;

	double xb = 0;
	double yb = 0;
	double zb = 0;

	double xp = 0;
	double yp = 0;
	double zp = 0;

	double qx = 0;
	double qy = 0;
	double qz = 0;
	double qw = 0;



	ofstream file_output; //output file for matlab
    file_output.open("box_db_pop", ofstream::app);

    ofstream file_output_1; //output file for matlab
    file_output_1.open("box_db_pop.csv", ofstream::app);


	///////////////////////////////// get values from file for each line //////////////////////////////////
	for(std::string line; getline( file, line, '\n' ); ) // for each line
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    		values_inline.push_back(stod(value));

    	if( values_inline[0] > 0)
    	{
    		q = values_inline[0];

    		xb = values_inline[1];
			yb = values_inline[2];
			zb = values_inline[3];

			xp = values_inline[4];
			yp = values_inline[5];
			zp = values_inline[6];

			qx = values_inline[7];
			qy = values_inline[8];
			qz = values_inline[9];
			qw = values_inline[10];


			Eigen::MatrixXd T_point = MatrixXd::Identity(4,4);

			T_point(0,3) = xp;
			T_point(1,3) = yp;
			T_point(2,3) = zp;

			KDL::Rotation Rot_point = Rotation::Quaternion(qx,qy,qz,qw);

			T_point(0,0) = Rot_point.data[0];
			T_point(0,1) = Rot_point.data[1];
			T_point(0,2) = Rot_point.data[2];
			T_point(1,0) = Rot_point.data[3];
			T_point(1,1) = Rot_point.data[4];
			T_point(1,2) = Rot_point.data[5];
			T_point(2,0) = Rot_point.data[6];
			T_point(2,1) = Rot_point.data[7];
			T_point(2,2) = Rot_point.data[8];


			Eigen::MatrixXd T_rot_same_face = MatrixXd::Identity(4,4);
			Eigen::MatrixXd T_rot_opposite1 = MatrixXd::Identity(4,4);
			Eigen::MatrixXd T_rot_opposite2 = MatrixXd::Identity(4,4);


			Eigen::MatrixXd R0 = MatrixXd::Identity(3,3);
			Eigen::MatrixXd R1 = MatrixXd::Identity(3,3);
			Eigen::MatrixXd R2 = MatrixXd::Identity(3,3);

		
			R1 << 1.0, 0.0, 0.0,
				0.0, cos(180.0*M_PI/180.0), -sin(180.0*M_PI/180.0),
				0.0, sin(180.0*M_PI/180.0), cos(180.0*M_PI/180.0);
	
	
			R0 << cos(180.0*(M_PI/180.0)), sin(180.0*(M_PI/180.0)), 0.0,
				-sin(180.0*(M_PI/180.0)), cos(180.0*(M_PI/180.0)), 0.0,
				0.0,				0.0, 				1.0;
	
			R2 << cos(180.0*(M_PI/180.0)),  0.0, sin(180.0*(M_PI/180.0)),
					        0.0  ,  1.0,    0.0 ,
				-sin(180.0*(M_PI/180.0)), 0.0, cos(180.0*(M_PI/180.0));
			


			T_rot_same_face.block<3,3>(0,0) = R0;
			T_rot_opposite1.block<3,3>(0,0) = R1;
			T_rot_opposite2.block<3,3>(0,0) = R2;

			cout << T_rot_same_face << endl;
			cout << T_rot_opposite1 << endl;
			cout << T_rot_opposite2 << endl;

			cout << T_point << endl;



   	
			Eigen::MatrixXd T0 = MatrixXd::Identity(4,4);
			Eigen::MatrixXd T1 = MatrixXd::Identity(4,4);
			Eigen::MatrixXd T2 = MatrixXd::Identity(4,4);


			T0 = T_rot_same_face * T_point; //around z
			T1 = T_rot_opposite1 * T_point; //around x
			T2 = T_rot_opposite2 * T_point; //around y

			cout << "-------------- T0 - Z - 1----------------"<< endl;
			cout << T0 << endl;
			cout << "-------------- T1 - X - 3----------------"<< endl;
			cout << T1 << endl;
			cout << "-------------- T2 - Y - 2----------------"<< endl;
			cout << T2 << endl;

			


			Eigen::Matrix3f TR0(3,3);
			Eigen::Matrix3f TR1(3,3);
			Eigen::Matrix3f TR2(3,3);
			Eigen::Matrix3f TRpoint(3,3);

			for(int i = 0 ; i <3 ; i++)
			{	for(int j = 0 ; j <3 ; j++)
				{ 
					TR0(i,j) = T0(i,j);
					TR1(i,j) = T1(i,j);
					TR2(i,j) = T2(i,j);
					TRpoint(i,j) = T_point(i,j);
				}
			}


			Eigen::Quaternionf q0(TR0);
			Eigen::Quaternionf q1(TR1);
			Eigen::Quaternionf q2(TR2);
			Eigen::Quaternionf qp(TRpoint);

			file_output<<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<xp<<' '<<yp<<' '<<zp<<' '<<qx<<' '<<qy<<' '<<qz<<' '<<qw<<endl;
			file_output<<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T0(0,3)<<' '<<T0(1,3)<<' '<<T0(2,3)<<' '<<q0.x()<<' '<<q0.y()<<' '<<q0.z()<<' '<<q0.w()<<endl;
			file_output<<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T1(0,3)<<' '<<T1(1,3)<<' '<<T1(2,3)<<' '<<q1.x()<<' '<<q1.y()<<' '<<q1.z()<<' '<<q1.w()<<endl;
			file_output<<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T2(0,3)<<' '<<T2(1,3)<<' '<<T2(2,3)<<' '<<q2.x()<<' '<<q2.y()<<' '<<q2.z()<<' '<<q2.w()<<endl;

			file_output_1<<xb<<','<<yb<<','<<zb<<','<<xp<<','<<yp<<','<<zp<<','<<qx<<','<<qy<<','<<qz<<','<<qw<<endl;
			file_output_1<<xb<<','<<yb<<','<<zb<<','<<T0(0,3)<<','<<T0(1,3)<<','<<T0(2,3)<<','<<q0.x()<<','<<q0.y()<<','<<q0.z()<<','<<q0.w()<<endl;
			file_output_1<<xb<<','<<yb<<','<<zb<<','<<T1(0,3)<<','<<T1(1,3)<<','<<T1(2,3)<<','<<q1.x()<<','<<q1.y()<<','<<q1.z()<<','<<q1.w()<<endl;
			file_output_1<<xb<<','<<yb<<','<<zb<<','<<T2(0,3)<<','<<T2(1,3)<<','<<T2(2,3)<<','<<q2.x()<<','<<q2.y()<<','<<q2.z()<<','<<q2.w()<<endl;



			cout <<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<xp<<' '<<yp<<' '<<zp<<' '<<qx<<' '<<qy<<' '<<qz<<' '<<qw<<endl;
			cout <<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T0(0,3)<<' '<<T0(1,3)<<' '<<T0(2,3)<<' '<<q0.x()<<' '<<q0.y()<<' '<<q0.z()<<' '<<q0.w()<<endl;
			cout <<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T1(0,3)<<' '<<T1(1,3)<<' '<<T1(2,3)<<' '<<q1.x()<<' '<<q1.y()<<' '<<q1.z()<<' '<<q1.w()<<endl;
			cout <<q<<' '<<xb<<' '<<yb<<' '<<zb<<' '<<T2(0,3)<<' '<<T2(1,3)<<' '<<T2(2,3)<<' '<<q2.x()<<' '<<q2.y()<<' '<<q2.z()<<' '<<q2.w()<<endl;
		}
    }
}