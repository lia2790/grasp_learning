-/*

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

//#include <grasp_learning/box_dimension.h>


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>


#include <cmath>
#include <ctime>
#include <time.h>


using namespace std;


ofstream file_output;
std::string relative_path_file_out;




void listen_box_callback(const grasp_learning::box_dimension box_dim)
{
	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");	

	std::string file_name_out = path + relative_path_file_out;
 	std::string name = "box_dimension";
  	file_output.open( file_name_out + name, ofstream::app);	

  	file_output<<box_dim.x<<' '<<box_dim.y<<' '<<box_dim.z<<' '<<endl;
}



int main (int argc, char **argv)
{

	ros::init(argc, argv, "comunication_vision");	// ROS node
	ros::NodeHandle nh;




	nh.param<std::string>("filename_out", relative_path_file_out, "/box_estimate/" );
	ros::Subscriber box_sub = nh.subscribe("vision_topic",1,&listen_box_callback);


	ros::Rate loop_rate(100);

	while(nh.ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}

	




	return 0;
}