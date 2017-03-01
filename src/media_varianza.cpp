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
#include "normal_component_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;


string relative_path_file;
string file_name;



int main (int argc, char **argv)
{

	ros::init(argc, argv, "media_varianza");	// ROS node
	ros::NodeHandle nh;

	nh.param<std::string>("file_name", relative_path_file, "/db/box_db_2.csv" );

	ofstream file_output; //output file
    file_output.open("box_db_quality.txt", ofstream::app);



	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp-learning");
	file_name = path + relative_path_file;
	ifstream file(file_name); 

	std::cout << "file: " << file_name.c_str() << " is " << (file.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file.is_open())
		return 0;





	std::vector<double> contact_wrenches;
	
	///////////////////////////////// get values from file for each line //////////////////////////////////
	for(std::string line; getline( file, line, '\n' ); ) // for each line
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ',' ); )
    		values_inline.push_back(stod(value));

    	for(int i = 0 ; i < 120 ; i++)
    		if( !std::isnan(values_inline[110 + i]))
   				contact_wrenches.push_back(values_inline[110 + i]); 		
    }







    Eigen::VectorXd f_i(3);
    std::vector<double> f_i_norm_total; // vettore delle norme delle forze

	int total = 0;
    int s = 0;

    for(int i = 0 ; i < (contact_wrenches.size()/6)  ; i++)
    {
    	f_i(0) = contact_wrenches[s+0];
    	f_i(1) = contact_wrenches[s+1];
    	f_i(2) = contact_wrenches[s+2];

    	f_i_norm_total.push_back(f_i.norm());
    	total++;
    	s += 6;
    }


    double max_f_i = 0;
    for(int i = 0; i < f_i_norm_total.size() ; i++)
        if(f_i_norm_total[i] > max_f_i)
            max_f_i = f_i_norm_total[i];


    cout << "MAX NORM FORCE : " << max_f_i << endl;



    double f_i_total = 0;
    for(int i = 0; i < f_i_norm_total.size() ; i++)
    	f_i_total += f_i_norm_total[i];


    double media = f_i_total / total ;


    double var = 0;
    for(int i = 0 ; i < f_i_norm_total.size() ; i++)
    	var += (f_i_norm_total[i] - media)*(f_i_norm_total[i] - media);

    double varianza = var / (f_i_norm_total.size() - 1) ;

    double sigma = sqrt(varianza);

    double f_i_max = media + 2*sigma;



    cout << "f_i_total : " << f_i_total << endl;
    cout << " total : " << total << endl;
    cout << " media : " << media << endl;
    cout << " sigma : " << sigma << endl;
    cout << "f_i_max : " << f_i_max << endl;


	ros::spinOnce();
	return 0;


}