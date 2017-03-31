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
////////////////////////////////////////////////


/////////////////////////////////  CONTACT POINT

Eigen::VectorXd cp(7);
////////////////////////////////////////////////



int main (int argc, char **argv)
{

	ros::init(argc, argv, "show_hand_point");	// ROS node
	ros::NodeHandle nh;



	string relative_path_file_in;	
	string file_name_in;

	string relative_path_file_hand;
	string file_name_hand;



	nh.param<std::string>("file_name_in", relative_path_file_in, "/box_test/test" );
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
	/////////////////////////////////////////////////////////////////////////////////////////////////






	
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
    //////////////////////////////////////////////////////////////////////////////////////////////////





  	////////////////////////////////////////////////////////////////////////////// TAKE GRASP POINT
    std::vector<std::vector<KDL::Frame>> f_point;
    std::vector<KDL::Frame> fcp;


    for(std::string line; getline( file_in, line, '\n' ); )
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    		values_inline.push_back(stod(value));


    	box(0) = values_inline[0];
    	box(1) = values_inline[1];
    	box(2) = values_inline[2];

    	double px = values_inline[3]; //px
		double py = values_inline[4]; //py
		double pz = values_inline[5]; //pz

		double qx = values_inline[6]; //qx
		double qy = values_inline[7]; //qy
		double qz = values_inline[8]; //qz
		double qw = values_inline[9]; //qw



		KDL::Vector t_cp(px,py,pz);
		KDL::Rotation R_cp = Rotation::Quaternion(qx,qy,qz,qw);

		KDL::Frame f_cp(R_cp,t_cp);

		fcp.push_back(f_cp);

		std::vector<KDL::Frame> f_hand;


		for(int i = 0; i < hp.size(); i++)
		{
			KDL::Vector t_hp(hp[i](0), hp[i](1), hp[i](2));
			KDL::Rotation R_hp = Rotation::Quaternion(hp[i](3),hp[i](4),hp[i](5),hp[i](6));
			KDL::Frame f_hp(R_hp, t_hp);


			KDL::Frame f_hp_cp = f_cp*f_hp;

			f_hand.push_back(f_hp_cp);
		}	

		f_point.push_back(f_hand);
    }


    /////////////////////////////    SHOW RVIZ    ///////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
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
    std::string stringaFrameIdPadre_box = "/world";
	std::string stringaFrameIdFiglio_box = "base_frame";

	tf::StampedTransform ObjToSurfaceBase(trasformazione, ros::Time::now(), stringaFrameIdPadre_box, stringaFrameIdFiglio_box);
	tf_broadcaster.sendTransform(ObjToSurfaceBase);
	/////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////////      show BOX
	// For visualizing things in rviz FOR SHOW BOX
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_box;
	visual_tools_box.reset(new rviz_visual_tools::RvizVisualTools("base_frame","/rviz_visual_markers_box"));

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
	//////////////////////////////////////////////////////////////////////////////////////////////// 


	ros::Rate loop_rate(1);


	while(nh.ok())
	{
		tf::StampedTransform ObjToSurfaceBase(trasformazione, ros::Time::now(), stringaFrameIdPadre_box, stringaFrameIdFiglio_box); // stamped always frame world
		tf_broadcaster.sendTransform(ObjToSurfaceBase);

		visual_tools_box->publishWireframeCuboid(pose_box,  x_depth,  y_width, z_height); // stamped always box
		//  //x_depth; // y_width // z_height


		/////////////////////////////////////////////////// contact frame
		for(int i = 0; i < f_point[0].size(); i++)
		{
			std::string stringaFrameIdPadre = "base_frame";
			std::string stringaFrameIdFiglio = "contact_frame_" + std::to_string(i);

			std::string stringaFrameIdPadre_ = "base_frame";
			std::string stringaFrameIdFiglio_ = "contact_frame_cp" ;

			double px_ = fcp[0].p.x();
			double py_ = fcp[0].p.y();
			double pz_ = fcp[0].p.z();
		
			double qx_ = 0;
			double qy_ = 0;
			double qz_ = 0;
			double qw_ = 1;

			fcp[0].M.GetQuaternion(qx_,qy_,qz_,qw_);



			tf::Quaternion rotazione_(qx_,qy_,qz_,qw_);
    		tf::Vector3 traslazione_(px_,py_,pz_);
    		tf::Transform trasformazione_(rotazione_, traslazione_);

			tf::StampedTransform ObjToSurfaceT(trasformazione_, ros::Time::now(), stringaFrameIdPadre_, stringaFrameIdFiglio_);
			tf_broadcaster.sendTransform(ObjToSurfaceT);





			double px = f_point[0][i].p.x();
			double py = f_point[0][i].p.y();
			double pz = f_point[0][i].p.z();
		
			double qx = 0;
			double qy = 0;
			double qz = 0;
			double qw = 1;

			f_point[0][i].M.GetQuaternion(qx,qy,qz,qw);
			

			cout << "Position: [ px : " << px << " , py : " << py << " , pz : " << pz << " ]" << endl;
			cout << "Orientation : [ qx : " << qx << " , qy : " << qy << " , qz : " << qz << " , qw : " << qw << " ]" << endl;

			tf::Quaternion rotazione(qx,qy,qz,qw);
    		tf::Vector3 traslazione(px,py,pz);
    		tf::Transform trasformazione(rotazione, traslazione);

			tf::StampedTransform ObjToSurface(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
			tf_broadcaster.sendTransform(ObjToSurface);
		}



		visual_tools_box->triggerBatchPublish();
		ros::spinOnce();
		loop_rate.sleep();	
	}




	ros::spinOnce();
	return 0;
}