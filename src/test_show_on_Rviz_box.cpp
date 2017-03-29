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
#include <eigen_conversions/eigen_msg.h>

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

Eigen::VectorXd box(3);
//////////////////////////////////////////////


/////////////////////////////////// CONTACT POINTS

Eigen::VectorXd contact_points(20*3);
///////////////////////////////////////


////////////////////////////////////  CONTACT FORCE

Eigen::VectorXd contact_wrenches(20*6);
/////////////////////////////////////////////


////////////////////////////////////  ALL CONTACT FLAG

Eigen::VectorXd contact_flag(20);
/////////////////////////////////////////////


////////////////////////////////////  CONTACT

std::vector<double> contact_id;
/////////////////////////////////////////////





int n_c;
string relative_path_file;
string file_name;


int main (int argc, char **argv)
{

	ros::init(argc, argv, "Visual_box_wrench");	// ROS node
	ros::NodeHandle nh;



	ros::Publisher Wrench_pub_center = nh.advertise<geometry_msgs::WrenchStamped>("/box_contact_wrench_center", 1);
	ros::Publisher Wrench_pub_contact = nh.advertise<geometry_msgs::WrenchStamped>("/box_contact_wrench_contact", 1);




	




	nh.param<std::string>("file_name", relative_path_file, "/db/test_file.csv" );
	//////////////////////////////////////////////////////////////////////////// 	TAKE DATA FROM DATABASE ----------- ONLY ONE LINE
	std::string path = ros::package::getPath("grasp_learning");
	file_name = path + relative_path_file;
	ifstream file(file_name); 

	string line;
	getline( file, line, '\n' ); //take only one line or rather one grasp
	
	std::vector<double> values_inline;
    std::istringstream iss_line(line);	
    for(std::string value; getline(iss_line, value, ',' ); )
    	values_inline.push_back(stod(value));


    box(0) = values_inline[0];
    box(1) = values_inline[1];
    box(2) = values_inline[2];

    for(int i = 0 ; i < 60 ; i++)
    	contact_points(i) = values_inline[50 + i];

   	for(int i = 0 ; i < 120 ; i++)
   		contact_wrenches(i) = values_inline[110 + i];
	////////////////////////////////////////////////////////////////////////////////////////////////////////
    





   	//////////////////////////////////////////////////////////////////////////////	IT CREATES THE DATA STRUCTURE 
   	for(int i = 0 ; i < 20 ; i++) // 0 : no contact
   		contact_flag(i) = 0;
   	

   	Eigen::MatrixXd cp(20,3);
   	int k = 0;
   	for(int i = 0 ; i < 20 ; i++)
   		for(int j=0 ; j < 3 ; j++)
		{	cp(i,j) = values_inline[50 + k]; if( !std::isnan(cp(i,j))) contact_flag(i) = 1; k++; }


	Eigen::MatrixXd cf(20,6);
   	int h = 0;
   	for(int i = 0 ; i < 20 ; i++)
   		for(int j=0 ; j < 6; j++)
		{	cf(i,j) = values_inline[110 + h]; h++; }



	for(int i = 0 ; i < 20 ; i++)
		if( contact_flag(i) )
			contact_id.push_back(i);


	cout << " cp : " << endl << cp << endl;
	cout << " cf : " << endl << cf << endl;
	cout << " contact_flag" << endl << contact_flag << endl;
	for(int i= 0; i < contact_id.size(); i++)
		cout << " contact_id : " << endl << contact_id[i] << endl;
	///////////////////////////////////////////////////////////////////////////////////////////////////////7
    






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
		// Publish arrow vector of pose
		// ROS_INFO_STREAM_NAMED("test","Publishing Box");

		tf::StampedTransform ObjToSurfaceBase(trasformazione, ros::Time::now(), stringaFrameIdPadre_, stringaFrameIdFiglio_); // stamped always frame world
		tf_broadcaster.sendTransform(ObjToSurfaceBase);

		visual_tools_->publishWireframeCuboid(pose_box,  x_depth,  y_width, z_height); // stamped always box
		//  //x_depth; // y_width // z_height


	
		/////////////////////////////////////////////////// contact frame
		for(int i= 0; i < contact_id.size(); i++)
		{
			std::string stringaFrameIdPadre = "base_frame";
			std::string stringaFrameIdFiglio = "contact_frame_" + std::to_string(contact_id[i]);


			Eigen::MatrixXd c_Rotation_o(3,3);
			normal_component(c_Rotation_o, box(0)/2, box(1)/2, box(2)/2 , cp(contact_id[i],0), cp(contact_id[i],1), cp(contact_id[i],2));


			Eigen::MatrixXd b_Rotation_c = c_Rotation_o.transpose();
			KDL::Rotation R( b_Rotation_c(0,0), b_Rotation_c(0,1),b_Rotation_c(0,2),b_Rotation_c(1,0),b_Rotation_c(1,1),b_Rotation_c(1,2),b_Rotation_c(2,0),b_Rotation_c(2,1),b_Rotation_c(2,2));


	

			double px = cp(contact_id[i],0);//p.translation[i];
			double py = cp(contact_id[i],1);//p.translation[1];
			double pz = cp(contact_id[i],2);//p.translation[2];
		
			double qx ;//p.quaternion[1];
			double qy ;//p.quaternion[2];
			double qz ;//p.quaternion[3];
			double qw ;//p.quaternion[i];


			R.GetQuaternion(qx, qy, qz, qw);


			tf::Quaternion rotazione(qx,qy,qz,qw);
    		tf::Vector3 traslazione(px,py,pz);
    		tf::Transform trasformazione(rotazione, traslazione);

			tf::StampedTransform ObjToSurface(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
			tf_broadcaster.sendTransform(ObjToSurface);



			Eigen::VectorXd fo(6);
			fo << -cf(contact_id[i],0), -cf(contact_id[i],1), -cf(contact_id[i],2), cf(contact_id[i],3), cf(contact_id[i],4), cf(contact_id[i],5);
			Eigen::VectorXd fc(6);
			// fc = b_Rotation_c*fo;

			Eigen::MatrixXd Grasp_Matrix = MatrixXd::Identity(6,6);
			Eigen::MatrixXd Skew_Matrix = MatrixXd::Zero(3,3);



	      	Skew_Matrix(0,0) = Skew_Matrix(1,1) = Skew_Matrix(2,2) = 0;
     		Skew_Matrix(0,1) = - cp(contact_id[i],2); // -rz    
     		Skew_Matrix(0,2) = cp(contact_id[i],1);   // ry
        	Skew_Matrix(1,0) = cp(contact_id[i],2);   // rz
        	Skew_Matrix(2,0) = - cp(contact_id[i],1); // -ry
        	Skew_Matrix(1,2) = - cp(contact_id[i],0); // -rx
       		Skew_Matrix(2,1) = cp(contact_id[i],0);   // rx

        	Grasp_Matrix.block<3,3>(0,0) = c_Rotation_o;
     		Grasp_Matrix.block<3,3>(3,3) = c_Rotation_o;
     		Grasp_Matrix.block<3,3>(3,0) = Skew_Matrix * c_Rotation_o;
        	Grasp_Matrix.block<3,3>(0,3) = MatrixXd::Zero(3,3);

			fc = Grasp_Matrix*fo;


			cout << " fo : " << endl << fo << endl;
			cout << "-----------------------" << endl;
			cout << " fc : " << endl << fc << endl;


			geometry_msgs::WrenchStamped wrMsg_;

			wrMsg_.header.frame_id = "base_frame";
			wrMsg_.header.stamp = ros::Time::now();


			wrMsg_.wrench.force.x = fo(0);
			wrMsg_.wrench.force.y = fo(1);
			wrMsg_.wrench.force.z = fo(2);

			wrMsg_.wrench.torque.x = fo(3);
			wrMsg_.wrench.torque.y = fo(4);
			wrMsg_.wrench.torque.z = fo(5);

			Wrench_pub_center.publish(wrMsg_); // force mapping in center of the box





			geometry_msgs::WrenchStamped wrMsg;

			wrMsg.header.frame_id = "contact_frame_" + std::to_string(contact_id[i]);
			wrMsg.header.stamp = ros::Time::now();


			wrMsg.wrench.force.x = fc(0);
			wrMsg.wrench.force.y = fc(1);
			wrMsg.wrench.force.z = fc(2);

			wrMsg.wrench.torque.x = fc(3);
			wrMsg.wrench.torque.y = fc(4);
			wrMsg.wrench.torque.z = fc(5);

			Wrench_pub_contact.publish(wrMsg);

		}
		
						
		

		visual_tools_->triggerBatchPublish();


		

		ros::spinOnce();
		loop_rate.sleep();

		
	}
	

	return 0;

}

