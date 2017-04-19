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



using namespace std;


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "Comunication");// ROS node
	ros::NodeHandle nn;


	string relative_path_file_in;	
	string file_name_in;
	nn.param<std::string>("filename_in", relative_path_file_in, "/bottiglia/test_itself/model_40_8/box_estimate" );




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
	


	

	geometry_msgs::Pose hand_pose;

	hand_pose.position.x = values_inline[4] - 0.8;
	hand_pose.position.y = values_inline[5];
	hand_pose.position.z = values_inline[6];

	hand_pose.orientation.x = values_inline[7];
	hand_pose.orientation.y = values_inline[8];
	hand_pose.orientation.z = values_inline[9];
	hand_pose.orientation.w = values_inline[10];



	ros::Rate loop_rate(100);

	while(nn.ok())
	{
		grasping_pub.publish(hand_pose);

		ros::spinOnce();
		loop_rate.sleep();

		cout << "grasp !" << endl;
		cout << "------------------------------------" << endl;

		cout << " x : " << hand_pose.position.x << " y : " << hand_pose.position.y << " z : " << hand_pose.position.z << endl;
		cout << " qx : " << hand_pose.orientation.x << " qy : " << hand_pose.orientation.y << " qz : " << hand_pose.orientation.z << " qw : " << hand_pose.orientation.w << endl;
		cout << "------------------------------------" << endl;

	} 
}