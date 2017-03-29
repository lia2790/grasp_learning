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


#include <rviz_visual_tools/rviz_visual_tools.h>


using namespace std;
using namespace Eigen;
using namespace KDL;



//////////////////////////////////	BOX

Eigen::VectorXd box(3);
//////////////////////////////////////////////



int main (int argc, char **argv)
{

	ros::init(argc, argv, "svm_predict");	// ROS node
	ros::NodeHandle nh;


	string relative_path_file_in;
	string file_name_in;
	nh.param<std::string>("file_name_out", relative_path_file_in, "/box_estimate/" );



	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");
	file_name_in = path + relative_path_file_in;
	ifstream file_in(file_name_in); 






	Eigen::VectorXd grasp = VectorXd::Zero(7);
	std::vector<Eigen::VectorXd> Grasp;


	//////////////////////////////////////////////////////////////////////////// 	TAKE DATA FROM DATABASE ----------- ONLY ONE LINE
	for(std::string line; getline( file_in, line, '\n' ); )
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    			values_inline.push_back(stod(value));

    	box(0) = values_inline[1];
    	box(1) = values_inline[2];
    	box(2) = values_inline[3];

    	if(values_inline[0]>0)
    	{
    		for(int i = 0 ; i < 7 ; i++)
    			grasp(i) = values_inline[4+i];

    		Grasp.push_back(grasp);
    	}	
    }
	////////////////////////////////////////////////////////////////////////////////////////////////////////







	
	// For visualizing things in rviz
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers"));


	geometry_msgs::Pose pose_;

	pose_.position.x = 0;
	pose_.position.y = 0;
	pose_.position.z = 0;

	pose_.orientation.x = 0;
	pose_.orientation.y = 0;
	pose_.orientation.z = 0;
	pose_.orientation.w = 1;
	
	double x_depth = box(0);
	double y_width = box(1);
	double z_height = box(2);
	


	Eigen::Quaterniond q_box(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
    Eigen::Translation3d t_box(pose_.position.x, pose_.position.y, pose_.position.z);
    Eigen::Affine3d pose_box = Eigen::Affine3d::Identity() * t_box * q_box;




	ros::Rate loop_rate(1);


	static tf::TransformBroadcaster tf_broadcaster; 

	//////////////////////////////////		trasf world
	double px = 0;
	double py = 0;
	double pz = 0;
		
	double qx = 0;
	double qy = 0;
	double qz = 0;
	double qw = 1;

	tf::Quaternion rotazione(qx,qy,qz,qw);
    tf::Vector3 traslazione(px,py,pz);
    tf::Transform trasformazione(rotazione, traslazione);
    std::string stringaFrameIdPadre_ = "/world";
	std::string stringaFrameIdFiglio_ = "base_frame";

	tf::StampedTransform ObjToSurfaceBase(trasformazione, ros::Time::now(), stringaFrameIdPadre_, stringaFrameIdFiglio_);
	tf_broadcaster.sendTransform(ObjToSurfaceBase);

	while(nh.ok())
	{
		tf::StampedTransform ObjToSurfaceBase(trasformazione, ros::Time::now(), stringaFrameIdPadre_, stringaFrameIdFiglio_); // stamped always frame world
		tf_broadcaster.sendTransform(ObjToSurfaceBase);

		visual_tools_->publishWireframeCuboid(pose_box,  x_depth,  y_width, z_height); // stamped always box
		//  //x_depth; // y_width // z_height

		/////////////////////////////////////////////////// contact frame
		for(int i= 0; i < Grasp.size(); i++)
		{
			std::string stringaFrameIdPadre = "base_frame";
			std::string stringaFrameIdFiglio = "contact_frame_" + std::to_string(i);

	

			double px = Grasp[i](0);
			double py = Grasp[i](1);
			double pz = Grasp[i](2);
		
			double qx = Grasp[i](3);
			double qy = Grasp[i](4);
			double qz = Grasp[i](5);
			double qw = Grasp[i](6);


			tf::Quaternion rotazione(qx,qy,qz,qw);
    		tf::Vector3 traslazione(px,py,pz);
    		tf::Transform trasformazione(rotazione, traslazione);

			tf::StampedTransform ObjToSurface(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
			tf_broadcaster.sendTransform(ObjToSurface);
		}

		visual_tools_->triggerBatchPublish();


		

		ros::spinOnce();
		loop_rate.sleep();	
	}
	

	return 0;
}

