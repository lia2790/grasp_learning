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


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>


#include <cmath>
#include <ctime>
#include <time.h>



using namespace std;
using namespace Eigen;



 
int number_box;         // How many boxes I want to Generate
double distance_hand;   // meters
int discretize_side;    // sampling of boxes

std::vector<double> box_input;


struct box                  
{                       
    double width  = 0; // x
	double height = 0; // y
	double length = 0; // z
};
int offset = 1; // centimeters
int interval_size = 100; // centimeters
// sampling interval from 1 cm (offset) to 100 cm (interval_size)


double quality = 0;
//I create a random quality index for each grasp a box
//random value da 0 a 100 (from offset to interval_size)


ofstream file_output; //output file for klampt
ofstream file_output1; //output file for matlab train svm



std::vector<Eigen::MatrixXd> populate_face(Eigen::Vector3d axis_dimensions, int disc, double dist_hand, Eigen::Matrix4d T_init)
{
    // std::vector<Eigen::Matrix4d> results((disc+1)*(disc+1)*6, Eigen::Matrix4d::Identity());
    /* The total size of the output vertor is disc*disc*(disc-)*6*/
    /* it is because each side of each box face is discretized by disc, then the angles are as well
    but PI an -PI is the same so the last is not evaluated, then there are 6 faces
    example if disc =3 ... 4*4*4*6    */


    std::vector<Eigen::MatrixXd> results;
    results.clear();


    Eigen::Matrix3d m, m_start;

    //  Plane YZ;
    for (double i = -1.0 * axis_dimensions(1) / 2 ; i <= axis_dimensions(1) / 2; i = i + (axis_dimensions(1) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(2) / 2 ; j <= axis_dimensions(2) / 2; j = j + (axis_dimensions(2) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = axis_dimensions(0) / 2 + dist_hand;
            start(1, 3) = i;
            start(2, 3) = j;
            m_start = Eigen::AngleAxisd( -M_PI / 2, Eigen::Vector3d::UnitY());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(0, 3) = -1.0 * (axis_dimensions(0) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitY());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }
    // Plane XZ;

    for (double i = -1.0 * axis_dimensions(0) / 2 ; i <= axis_dimensions(0) / 2; i = i + (axis_dimensions(0) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(2) / 2 ; j <= axis_dimensions(2) / 2; j = j + (axis_dimensions(2) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = i;
            start(1, 3) = axis_dimensions(1) / 2 + dist_hand;
            start(2, 3) = j;
            m_start = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(1, 3) = -1.0 * (axis_dimensions(1) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( -M_PI / 2, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }

    // Plane XY

    for (double i = -1.0 * axis_dimensions(0) / 2 ; i <= axis_dimensions(0) / 2; i = i + (axis_dimensions(0) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(1) / 2 ; j <= axis_dimensions(1) / 2; j = j + (axis_dimensions(1) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = i;
            start(1, 3) = j;
            start(2, 3) = axis_dimensions(2) / 2 + dist_hand;
            m_start = Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(2, 3) = -1.0 * (axis_dimensions(2) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }
    return results;

}




int main (int argc, char **argv)
{

	ros::init(argc, argv, "box_poses");	// ROS node
	ros::NodeHandle nh;


    nh.param<int>("number_box",number_box,1);
    nh.param<int>("discrete_side",discretize_side,2);
    nh.param<double>("distance_hand",distance_hand,0.005);
    nh.param<std::vector<double>>("box_input",box_input,{0.1,0.1,0.1});


    string file_name_out;
    string relative_path_file_out;
    nh.param<std::string>("filename_out", relative_path_file_out, "/box_estimate/" );

    srand(time(NULL)); //se non voglio la stessa sequenza di numeri casuali
	

    ofstream file_output; //output file 
    ofstream file_output1; //output file 

    std::string path = ros::package::getPath("grasp_learning");

    file_name_out = path + relative_path_file_out;
    

    std::string discr_side = std::to_string(discretize_side);
    std::string name1 = "box_db_" + discr_side;
    std::string name  ="box_db_" + discr_side + "_.csv";
    file_output1.open( file_name_out + name1, ofstream::app);
    file_output.open( file_name_out + name, ofstream::app);

    
	box n_box[number_box];  //array di box

    

	for(int i = 0; i < number_box ; i++)  //per ogni box individuata dall'indice i-esimo vado a discretizzare la superficie
	{
        n_box[i].width  = box_input[0]; // x
		n_box[i].height = box_input[1]; // y
		n_box[i].length = box_input[2]; // z
	
	   Eigen::Vector3d axis_dimensions_box(3);

	   axis_dimensions_box << n_box[i].width , n_box[i].height , n_box[i].length; // dimensioni della box   

	   Eigen::Matrix4d T_fixed_frame(4,4);
	   T_fixed_frame << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // nessuna rotazione nessuna traslazione 
                                                                        // corrisponde al centro della box


  
	   std::vector<Eigen::MatrixXd> grasp_point = populate_face(axis_dimensions_box, discretize_side, distance_hand, T_fixed_frame );


    

		for(int j = 0 ; j < grasp_point.size(); j++)
		{
            

            Matrix3f Rotation_matrix(3,3);
            Rotation_matrix << grasp_point[j](0,0) , grasp_point[j](0,1) , grasp_point[j](0,2),
                               grasp_point[j](1,0) , grasp_point[j](1,1) , grasp_point[j](1,2),
                               grasp_point[j](2,0) , grasp_point[j](2,1) , grasp_point[j](2,2);
            Quaternionf q(Rotation_matrix);

            q.normalize();

 
            /* OUTput KLAMPT */


            file_output 
            <<n_box[i].width<<','<<n_box[i].height<<','<<n_box[i].length<<','                // box h l d
            <<grasp_point[j](0,3)<<','<<grasp_point[j](1,3)<<','<<grasp_point[j](2,3)<<','   // x y z
            <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();  
            file_output << endl; 

            /* output for matlab */

            file_output1 
            <<n_box[i].width<<' '<<n_box[i].height<<' '<<n_box[i].length<<' '                // box h l d
            <<grasp_point[j](0,3)<<' '<<grasp_point[j](1,3)<<' '<<grasp_point[j](2,3)<<' '   // x y z
            <<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w();  
            file_output1 << endl; 

            
        
        }
	}// end for


    file_output.close();
    file_output1.close();

	cout << " END " << endl;



	ros::shutdown();
    ros::spin();
	return 0;
}