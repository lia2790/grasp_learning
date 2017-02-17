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


#include <rviz_visual_tools/rviz_visual_tools.h>


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

#include "geometry_msgs/WrenchStamped.h"
#include "normal_component_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;




//////////////////////////////////	BOX

std::vector<double> box;
//////////////////////////////////////////////


/////////////////////////////////// CONTACT POINTS

std::vector<double> contact_points;
///////////////////////////////////////


////////////////////////////////////  CONTACT FORCE

std::vector<double> contact_wrenches;
/////////////////////////////////////////////

int n_c;





int main (int argc, char **argv)
{

	ros::init(argc, argv, "Visual_box_wrench");	// ROS node
	ros::NodeHandle nh;


	
	nh.param<std::vector<double>>("box", box, std::vector<double>{1, 1, 1});
	nh.param<std::vector<double>>("contact_points", contact_points, std::vector<double>{1,1,1,1,1,1});
	nh.param<std::vector<double>>("contact_wrenches", contact_wrenches, std::vector<double>{1,1,1,1,1,1});
	nh.param<int>("n_c", n_c, 1);










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
	
	double x_depth = box[0];
	double y_width = box[1];
	double z_height = box[2];


	// Publish arrow vector of pose
	ROS_INFO_STREAM_NAMED("test","Publishing Box");
	visual_tools_->publishCuboid(pose_,  x_depth,  y_width, z_height); //x_depth; // y_width // z_height




	ros::Publisher Wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/box_contact_wrench", 1);



	for(int j = 0 ; j < n_c ; j++)
	{
		geometry_msgs::WrenchStamped wrMsg;

		wrMsg.header.frame_id = "contact_frame";
		wrMsg.header.stamp = ros::Time::now();

		wrMsg.wrench.force.x = contact_wrenches[0];
		wrMsg.wrench.force.y = contact_wrenches[1];
		wrMsg.wrench.force.z = contact_wrenches[2];

		wrMsg.wrench.torque.x = contact_wrenches[3];
		wrMsg.wrench.torque.y = contact_wrenches[4];
		wrMsg.wrench.torque.z = contact_wrenches[5];

		Wrench_pub.publish(wrMsg);
	}




	

				
	//creiamo un legame fra i sistemi di riferimento
	std::string stringaFrameIdPadre = "base_frame";
	std::string stringaFrameIdFiglio = "contact_frame";


	tf::TransformBroadcaster tf_broadcaster; 




	Eigen::MatrixXd b_Rotation_c(3,3);
	normal_component(b_Rotation_c, box[0]/2, box[1]/2, box[2]/2 , contact_points[0], contact_points[1], contact_points[2]);



	Eigen::Quaternion q(b_Rotation_c);


	double px = contact_points[0];//p.translation[0];
	double py = contact_points[1];//p.translation[1];
	double pz = contact_points[2];//p.translation[2];
	
	double qx = q.x();//p.quaternion[1];
	double qy = q.y();//p.quaternion[2];
	double qz = q.z();//p.quaternion[3];
	double qw = q.w();//p.quaternion[0];


	tf::Quaternion rotazione(qx,qy,qz,qw);
    tf::Vector3 traslazione(px,py,pz);
    tf::Transform trasformazione(rotazione, traslazione);

	tf::StampedTransform ObjToSurface(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
	tf_broadcaster.sendTransform(ObjToSurface);
	

	ros::spinOnce();
	return 0;

}

