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


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>

#include <math.h>
#include <stdio.h>
#include <ctime>
#include <time.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

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


using namespace KDL;
using namespace Eigen;
using namespace std;

Eigen::VectorXf Eigen_error;

void cb_control_error(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for (int i=0;i<msg->data.size();++i)
		Eigen_error(i) = msg->data[i];
}


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "Comunication");// ROS node
	ros::NodeHandle nn;


	string relative_path_file_in, on_topic, synergy_joint;	
	string file_name_in;
	nn.param<std::string>("filename_in", relative_path_file_in, "/bottiglia/test_itself/model_40_8/box_estimate" );
  	nn.param <std::string>("on_topic", on_topic, "/right_hand/joint_trajectory_controller/command");
  	nn.param <std::string>("synergy_joint", synergy_joint, "right_hand_synergy_joint");

	ros::Subscriber sub_error = nn.subscribe("/right_arm/teleoperation_controller/error", 1, cb_control_error);
	ros::Publisher hand_publisher = nn.advertise<trajectory_msgs::JointTrajectory>(on_topic.c_str(), 1000);


	Eigen_error = Eigen::VectorXf::Zero(6);
	Eigen_error(0) = 1000;

	ros::Publisher grasping_pub;
	grasping_pub = nn.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command", 1);

	




	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");


	file_name_in = path + relative_path_file_in;//input box 
	ifstream file_in(file_name_in); 


	std::cout << "file: " << file_name_in.c_str() << " is " << (file_in.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_in.is_open())
	return 0;
	//////////////////////////////////////////////////////////////////////////////////
	

	///////////////////// take a first pose ////////////////////////////////////	
  	std::string line; 
  	getline( file_in, line, '\n' );
	
	std::vector<double> values_inline;
    std::istringstream iss_line(line);	
    for(std::string value; getline(iss_line, value, ' ' ); )
    	values_inline.push_back(stod(value));
    //////////////////////////////////////////////////////////////////////////////////
	


	double dx = -1; 
	double dy = 0.0;
	double dz = values_inline[3]/2;// -0.02 + 0.05;
    Eigen::MatrixXd transf_World2Box =  MatrixXd::Identity(4,4);

    transf_World2Box(3,0) = dx;
    transf_World2Box(3,1) = dy;
    transf_World2Box(3,2) = dz;


    Eigen::MatrixXd transf_Box2Hand =  MatrixXd::Identity(4,4);
    transf_Box2Hand(3,0) = values_inline[4];
	transf_Box2Hand(3,1) = values_inline[5];
	transf_Box2Hand(3,2) = values_inline[6];
	// quaternion w y z x in EIGEN initialization
	Eigen::Quaterniond q_t(values_inline[10],values_inline[7],values_inline[8],values_inline[9]);


	Eigen::MatrixXd Rot_B2H = MatrixXd::Identity(3,3);
	Rot_B2H = q_t.toRotationMatrix();

	transf_Box2Hand.block<3,3>(0,0) = Rot_B2H;


	Eigen::MatrixXd World2Hand = transf_Box2Hand*transf_World2Box;	
	KDL::Rotation Rot(World2Hand(0,0)
	,World2Hand(0,1)
	,World2Hand(0,2)
	,World2Hand(1,0)
	,World2Hand(1,1)
	,World2Hand(1,2)
	,World2Hand(2,0)
	,World2Hand(2,1)
	,World2Hand(2,2));

	double qx = 10;
	double qy = 10; 
	double qz = 10; 
	double qw = 10;

	KDL::Rotation Rota;
	Rota = Rota*Rot;
	Rota.GetQuaternion(qx,qy,qz,qw);

	Eigen::Matrix3d m3;
	m3 << World2Hand(0,0)
	,World2Hand(0,1)
	,World2Hand(0,2)
	,World2Hand(1,0)
	,World2Hand(1,1)
	,World2Hand(1,2)
	,World2Hand(2,0)
	,World2Hand(2,1)
	,World2Hand(2,2);

	Eigen::Quaterniond q3(m3);
	 
	cout << "Rot_W2H : " << Rota << endl;
	cout << "Rot_B2H : " << Rot_B2H << endl;
	cout << " -----* qx : " << qx << " qy : " << qy << " qz : " << qz << " qw : " << qw << " ----- "<< endl;
	cout << " -----* qx : " << q3.x() << " qy : " << q3.y() << " qz : " << q3.z() << " qw : " << q3.w() << " ----- "<< endl;


	geometry_msgs::Pose hand_pose;

	hand_pose.position.x = World2Hand(3,0);
	hand_pose.position.y = World2Hand(3,1);
	hand_pose.position.z = World2Hand(3,2);

	hand_pose.orientation.x = q3.x();
	hand_pose.orientation.y = q3.y();
	hand_pose.orientation.z = q3.z();
	hand_pose.orientation.w = q3.w();



	ros::Rate loop_rate(100);

	float e_treshold(0.01);
	float error(100);
	grasping_pub.publish(hand_pose);
	ros::spinOnce();
	loop_rate.sleep();
	hand_pose.position.z = hand_pose.position.z + 0.3;
	while(nn.ok() && error > e_treshold)
	{
		grasping_pub.publish(hand_pose);

		ros::spinOnce();
		loop_rate.sleep();

		error = Eigen_error.norm();
		std::cout << "error: " << error << std::endl;
		// cout << "grasp !" << endl;
		// cout << "------------------------------------" << endl;
		// cout << " x : " << hand_pose.position.x << " y : " << hand_pose.position.y << " z : " << hand_pose.position.z << endl;
		// cout << " qx : " << hand_pose.orientation.x << " qy : " << hand_pose.orientation.y << " qz : " << hand_pose.orientation.z << " qw : " << hand_pose.orientation.w << endl;
		// cout << "------------------------------------" << endl;
	}
	std::cout << "Approaching" << std::endl;
	sleep(2.0);
	error = 1000;
	Eigen_error(0) = 1000;
	e_treshold = 0.0001;

	hand_pose.position.x = hand_pose.position.x - 0.0005;
	hand_pose.position.y = hand_pose.position.y - 0.0005;
	hand_pose.position.z = hand_pose.position.z - .3 - 0.02;
	while(nn.ok() && error > e_treshold)
	{
		grasping_pub.publish(hand_pose);

		ros::spinOnce();
		loop_rate.sleep();

		error = Eigen_error.norm();
		std::cout << "error: " << error << std::endl;
		// cout << "grasp !" << endl;
		// cout << "------------------------------------" << endl;
		// cout << " x : " << hand_pose.position.x << " y : " << hand_pose.position.y << " z : " << hand_pose.position.z << endl;
		// cout << " qx : " << hand_pose.orientation.x << " qy : " << hand_pose.orientation.y << " qz : " << hand_pose.orientation.z << " qw : " << hand_pose.orientation.w << endl;
		// cout << "------------------------------------" << endl;
	}

	trajectory_msgs::JointTrajectory msg_jointT_hand;
    msg_jointT_hand.header.stamp = ros::Time::now();
    msg_jointT_hand.points.resize(1);
    msg_jointT_hand.joint_names.resize(1);
    msg_jointT_hand.points[0].positions.resize(1);
    msg_jointT_hand.points[0].positions[0] = 1.0;
    msg_jointT_hand.points[0].velocities.resize(1);
    msg_jointT_hand.points[0].velocities[0] = 0.0;
    msg_jointT_hand.points[0].accelerations.resize(1);
    msg_jointT_hand.points[0].accelerations[0] = 0.0;
    msg_jointT_hand.points[0].effort.resize(1);
    msg_jointT_hand.points[0].effort[0] = 0.0;
    msg_jointT_hand.points[0].time_from_start = ros::Duration(10.0); // 10s;
    msg_jointT_hand.joint_names[0] = synergy_joint.c_str();
    hand_publisher.publish(msg_jointT_hand);
    ros::spinOnce();
    sleep(30.0);
    hand_pose.position.x = World2Hand(3,0);
	hand_pose.position.y = World2Hand(3,1);
	hand_pose.position.z = World2Hand(3,2) + 0.2;
	hand_pose.orientation.x = q3.x();
	hand_pose.orientation.y = q3.y();
	hand_pose.orientation.z = q3.z();
	hand_pose.orientation.w = q3.w();
    grasping_pub.publish(hand_pose);
    ros::spinOnce();
    sleep(30.0);
	// close hand
	trajectory_msgs::JointTrajectory msg_joint_T_hand;
    msg_joint_T_hand.header.stamp = ros::Time::now();
    msg_joint_T_hand.points.resize(1);
    msg_joint_T_hand.joint_names.resize(1);
    msg_joint_T_hand.points[0].positions.resize(1);
    msg_joint_T_hand.points[0].positions[0] = 0.0;
    msg_joint_T_hand.points[0].velocities.resize(1);
    msg_joint_T_hand.points[0].velocities[0] = 0.0;
    msg_joint_T_hand.points[0].accelerations.resize(1);
    msg_joint_T_hand.points[0].accelerations[0] = 0.0;
    msg_joint_T_hand.points[0].effort.resize(1);
    msg_joint_T_hand.points[0].effort[0] = 0.0;
    msg_joint_T_hand.points[0].time_from_start = ros::Duration(3.0); // 1s;
    msg_joint_T_hand.joint_names[0] = synergy_joint.c_str();
    hand_publisher.publish(msg_joint_T_hand);
    ros::spin();
}