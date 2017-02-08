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



#include "quality.h"
#include "quality_PCR_PGR_2.h"




using namespace std;
using namespace Eigen;
using namespace KDL;






// take it from yaml file
int n_rows;
int n_cols;
int quality_index;
string file_name;
string relative_path_file;
string frame_name_finger[5];
string frame_name_root;
//std::string root_name = "right_hand_softhand_base";
//std::string root_name = "right_hand_palm_link";
//std::string root_name = "world";
int n_fingers;
double joint_stiffness = 0;
double contact_stiffness = 0;
int type_of_contact = 0;
int n_q = 39;
int n_c_max = 20;

double quality_i = 0;


double mu = 0.03;
double f_i_max = 1;
int set_c = 0;


std::vector<double> synergie;



int main (int argc, char **argv)
{

	ros::init(argc, argv, "Grasp_quality");	// ROS node
	ros::NodeHandle nh;



 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	nh.param<int>("n_rows_file",n_rows,108);
	nh.param<int>("n_cols_file",n_cols,86);
	nh.param<int>("quality_index",quality_index,0);
	nh.param<int>("number_of_fingers", n_fingers, 5);
	nh.param<std::string>("frame_name_root", frame_name_root, "world");
	nh.param<std::string>("frame_name_thumb", frame_name_finger[0], "right_hand_index_distal_link" );
	nh.param<std::string>("frame_name_index", frame_name_finger[1], "right_hand_little_distal_link" );
	nh.param<std::string>("frame_name_middle", frame_name_finger[2], "right_hand_middle_distal_link");
	nh.param<std::string>("frame_name_ring", frame_name_finger[3], "right_hand_ring_distal_link");
	nh.param<std::string>("frame_name_little", frame_name_finger[4], "right_hand_thumb_distal_link");
	nh.param<std::string>("file_name", relative_path_file, "/db/box_db_2.csv" );
	nh.param<int>("type_of_contact", type_of_contact, 0);
	nh.param<double>("joint_stiffness",joint_stiffness,0);
	nh.param<double>("contact_stiffness",contact_stiffness,0);
	nh.param<std::vector<double>>("synergie", synergie, std::vector<double>{1});
	nh.param<double>("mu", mu, 0.03);
	nh.param<double>("f_i_max", f_i_max, 1);
	nh.param<int>("set_c", set_c, 0);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////


	

	ofstream file_output; //output file
    file_output.open("box_db_quality.txt", ofstream::app);








	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");
	file_name = path + relative_path_file;
	ifstream file(file_name); 

	std::cout << "file: " << file_name.c_str() << " is " << (file.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file.is_open())
	return 0;











	
	///////////////////// laod the urdf model //////////////////////////////////
	KDL::Tree hand_tree;
	KDL::Chain chains_hand_finger[n_fingers];
	KDL::Jacobian hand_jacob[n_fingers];
	KDL::JntArray q_finger[n_fingers];
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver[n_fingers];



	std::string robot_desc_string;
	nh.param("robot_description", robot_desc_string, string());  // robot description is the name in the launch file 
	if (!kdl_parser::treeFromString(robot_desc_string, hand_tree))
		{ ROS_ERROR("Failed to construct kdl tree"); return false;}
  

	for(int i=0; i < n_fingers; i++) // get chain for each fingers
	{	
		hand_tree.getChain(frame_name_root, frame_name_finger[i], chains_hand_finger[i]);      
		q_finger[i] = JntArray(chains_hand_finger[i].getNrOfJoints());
		jnt_to_jac_solver[i].reset(new KDL::ChainJntToJacSolver(chains_hand_finger[i]));
		q_finger[i].resize(chains_hand_finger[i].getNrOfJoints());
		hand_jacob[i].resize(chains_hand_finger[i].getNrOfJoints());

		cout << "JntArray " << i << " : " << chains_hand_finger[i].getNrOfJoints() << endl;
	}











///////////////////////////////// get values from file for each line //////////////////////////////////

	int first_element_joint_array = 17;
	int number_of_joints = 33;
	int first_element_cp_array = first_element_joint_array + number_of_joints;

	int count = 0; 
	int count_contact = 0;
	int count_line = 0;

	
	for(std::string line; getline( file, line, '\n' ); ) // for each line
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ',' ); )
    		values_inline.push_back(stod(value));

    	int quante_colonne = 0;

 /*   	for(int i = 0 ; i < values_inline.size(); i++)
    	{	cout << " line " << count_line << " : colonna : " << quante_colonne << " = " << values_inline[i] << endl; quante_colonne++;}
    		
    	cout << " numero di colonne : " << quante_colonne << endl;
 */   	
    	KDL::Vector trasl_w_T_o(0,0, (values_inline[2]/2)); // i m not sure if it is corrected
   		KDL::Vector trasl_o_T_p(values_inline[3],values_inline[4],values_inline[5]);
    	KDL::Rotation R_o_T_p = Rotation::Quaternion(values_inline[6],values_inline[7],values_inline[8],values_inline[9]);
   	
    	KDL::Frame w_T_o(trasl_w_T_o);
    	KDL::Frame o_T_p(R_o_T_p,trasl_o_T_p);
    	KDL::Frame p_T_h(chains_hand_finger[0].getSegment(5).getFrameToTip());

		KDL::Frame w_T_h = w_T_o * o_T_p * p_T_h ;
    	KDL::Vector trasl_w_T_h = w_T_h.p;
    	KDL::Rotation R_w_T_h = w_T_h.M;

    	double roll , pitch , yaw ;

    	R_w_T_h.GetRPY(roll,pitch,yaw);


    	int k_ = 0;

    	for(int i = 0; i < n_fingers; i++) //joint values
    	{	
    		q_finger[i](0) = trasl_w_T_h.x();
    		q_finger[i](1) = trasl_w_T_h.y();
    		q_finger[i](2) = trasl_w_T_h.z();
    		q_finger[i](3) = roll;
    		q_finger[i](4) = pitch;
    		q_finger[i](5) = yaw;


    		if(i == 4) // thumb
    		{
    			q_finger[i](6) = values_inline[first_element_joint_array]; // joint around z-axis knuckle

				q_finger[i](7) = values_inline[first_element_joint_array+1] ; // joint around y-axis knuckle
				q_finger[i](8) = values_inline[first_element_joint_array+2] ; // joint around y-axis proximal

				q_finger[i](9) = values_inline[first_element_joint_array+3] ; // joint around y-axis proximal
				q_finger[i](10) = values_inline[first_element_joint_array+4] ; // joint around y-axis distal

				k_+=5;
			}
			else
			{
				q_finger[i](6) = values_inline[first_element_joint_array+k_]; // joint around z-axis knuckle

				q_finger[i](7) = values_inline[first_element_joint_array+k_+1] ; // joint around y-axis knuckle
				q_finger[i](8) = values_inline[first_element_joint_array+k_+2] ; // joint around y-axis proximal

				q_finger[i](9) = values_inline[first_element_joint_array+k_+3] ; // joint around y-axis proximal
				q_finger[i](10) = values_inline[first_element_joint_array+k_+4] ;  // joint around y-axis middle

				q_finger[i](11) = values_inline[first_element_joint_array+k_+5] ;  // joint around y-axis middle
				q_finger[i](12) = values_inline[first_element_joint_array+k_+6] ;  // joint around y-axis distal

				k_+=7;
				}			
		}
/*		for ( int i = 0 ; i < n_fingers ; i++)
		{	cout << " finger " << i << endl;
			cout << q_finger[i].data << endl; }
*/
		string n_a_n = "nan";

		Eigen::MatrixXd Contacts(n_c_max,3);
		int gap_coordinate = 0;

		int n_c = 0;

		for(int i = 0; i < n_c_max; i++)
		{						
			for(int j = 0; j < 3; j++ )
				Contacts(i,j) = values_inline[first_element_cp_array + gap_coordinate + j];  // check if it is correct
			gap_coordinate += 3;
		}

		cout << "Contacts : " << endl;
		cout << Contacts << endl;


		for(int i = 0; i < n_c_max; i++)
			if(!std::isnan(Contacts(i,0)))	
				n_c++;
		
		if(n_c > 0)
		{
    			//for each contact point 
			Eigen::MatrixXd Grasp_Matrix_  = MatrixXd::Zero(6,6*n_c) ;  // G
    		Eigen::MatrixXd Hand_Jacobian_ = MatrixXd::Zero(6*n_c, n_q) ; // J

   			int k = 1;
   			int step = 0;		
			for(int i = 0 ; i < n_c_max ; i++) //calc the grasp_matrix and hand_jacobian for each contact point
    		{
    			if(!std::isnan(Contacts(i,0)))	
    			{
        			Eigen::MatrixXd Grasp_Matrix(6,6);  
 					Eigen::MatrixXd Skew_Matrix(3,3);
  					Eigen::MatrixXd Rotation(3,3);

  					Rotation <<  MatrixXd::Identity(3,3); 

	 				//check if the values ​​of the skew matrix are expressed in the correct reference system
    	  			Skew_Matrix(0,0) = Skew_Matrix(1,1) = Skew_Matrix(2,2) = 0;
     				Skew_Matrix(0,1) = - Contacts(i,2); // -rz    
     				Skew_Matrix(0,2) = Contacts(i,1);   // ry
        			Skew_Matrix(1,0) = Contacts(i,2);   // rz
        			Skew_Matrix(2,0) = - Contacts(i,1); // -ry
        			Skew_Matrix(1,2) = - Contacts(i,0); // -rx
       		 		Skew_Matrix(2,1) = Contacts(i,0);   // rx

        			Grasp_Matrix.block<3,3>(0,0) = Rotation;
     		   		Grasp_Matrix.block<3,3>(3,3) = Rotation;
     		   		Grasp_Matrix.block<3,3>(3,0) = Skew_Matrix * Rotation;
        			Grasp_Matrix.block<3,3>(0,3) = MatrixXd::Zero(3,3);



	      			Grasp_Matrix_.block<6,6>(0,step) = Grasp_Matrix;
       			


       				for(int n = 0 ; n < n_fingers ; n++)//calc jacobian for each finger
       					jnt_to_jac_solver[n]->JntToJac(q_finger[n], hand_jacob[n], -1);



    				int which_finger = i/3;
      				int which_falange = 0;
      				if(i<3)//thumb
        			{
       		 			if (i%3 == 0) which_falange = 6;
        				if (i%3 == 1) which_falange = 8;
        				if (i%3 == 2) which_falange = 10;
        			}
        			if(i>=3 && i<=14) 
        			{
        				if (i%3 == 0) which_falange = 8;
        				if (i%3 == 1) which_falange = 10;
        				if (i%3 == 2) which_falange = 12;
        			}
        			if(i > 14) // palm
	       			{	
	       	   			which_falange = 5;
	       			   	which_finger = 0;
	           		}
	           
/*	           		cout << "which_finger : " << which_finger << endl;
	        		cout << "which_falange : " << which_falange << endl;
*/
	           		jnt_to_jac_solver[which_finger]->JntToJac(q_finger[which_finger],hand_jacob[which_finger], which_falange);
        		
/*	  				cout << " Jacobian thumb "  << endl;
					cout << hand_jacob[0].data  << endl;
  					cout << " Jacobian index "  << endl;
	  				cout << hand_jacob[1].data  << endl;
			  		cout << " Jacobian middle " << endl;
	  				cout << hand_jacob[2].data  << endl;
					cout << " Jacobian ring "   << endl;
					cout << hand_jacob[3].data  << endl;
					cout << " Jacobian little " << endl;
					cout << hand_jacob[4].data  << endl;
					cout << " _______________ " << endl;
*/
					Hand_Jacobian_.block<6,6>(step,0) = hand_jacob[0].data.topLeftCorner(6,6);	// 6 
					Hand_Jacobian_.block<6,7>(step,6) = hand_jacob[0].data.topRightCorner(6,7); // 7
					Hand_Jacobian_.block<6,7>(step,13)= hand_jacob[1].data.topRightCorner(6,7); // 7
					Hand_Jacobian_.block<6,7>(step,20)= hand_jacob[2].data.topRightCorner(6,7); // 7
					Hand_Jacobian_.block<6,7>(step,27)= hand_jacob[3].data.topRightCorner(6,7); // 7
	  				Hand_Jacobian_.block<6,5>(step,34)= hand_jacob[4].data.topRightCorner(6,5); // 5

	  				step += 6;
	  			} // if i have contact
        	} // end for(n_max) each possible contact point	
        	cout << "................................................................." << endl;
        	cout << "FINAL GRASP MATRIX  " << endl;
        	cout << Grasp_Matrix_ << endl;
        	cout << "................................................................." << endl;
    
//////////////////////////	  now i have a 
/////////////////////////////   G          grasp matrix and 
/////////////////////////////   J          hand jacobian for 6D
/////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	        if(quality_index < 6) quality_i = quality(quality_index, Grasp_Matrix_, Hand_Jacobian_); 		

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    		if(quality_index == 6 ) // PCR PGR 
    		{
	    		int first_element_contact_force = 110;
	    		int number_force_contact = 120;

   				Eigen::VectorXd contact_force_ = VectorXd::Zero(6*n_c_max);
   				Eigen::VectorXd contact_force  = VectorXd::Zero(6*n_c);
   				int  j= 0;
    			for(int i = 0 ; i < number_force_contact ; i++)
    				if(!std::isnan(values_inline[first_element_contact_force + i]))
    				{	contact_force_(j) = (values_inline[first_element_contact_force + i]); j++; }

    			for(int i = 0 ; i < (6*n_c) ; i++)
    				contact_force(i) = contact_force_(i);
    				
				cout << "Contact_forces : " << endl;
    			cout << contact_force_ << endl;
    			cout << "______________" << endl;
    			cout << contact_force << endl;

    			Eigen::MatrixXd Contact_Stiffness_Matrix = MatrixXd::Zero(3,3);		// Kis
    			Eigen::MatrixXd Joint_Stiffness_Matrix = MatrixXd::Zero(n_q,n_q);    // Kp    				
    			for(int i = 0 ; i < Contact_Stiffness_Matrix.rows() ; i++) //Kis
    				Contact_Stiffness_Matrix(i,i) = contact_stiffness;

    			for(int j = 0 ; j < Joint_Stiffness_Matrix.rows() ; j++) //Kp
    				Joint_Stiffness_Matrix(j,j) = joint_stiffness;

    			int n_z = synergie.size();
    			Eigen::MatrixXd S(n_q, n_z);
    			for(int i = 0 ; i < n_q ; i++)
    				for(int j = 0 ; j < n_z ; j++)
    					S(i,j) = 1;

    			Eigen::VectorXd synergie_(synergie.size()) ;
    			for(int i = 0 ; i < synergie.size() ; i++)
    				synergie_(i) = synergie[i];

    			quality_i = quality_measures_PCR_PGR(contact_force, Grasp_Matrix_, Hand_Jacobian_, Contact_Stiffness_Matrix, Joint_Stiffness_Matrix, S, synergie_, mu, f_i_max, set_c);
    		}
    		else
    			quality_i = -33;
    	}// end if ( n_c > 0)

	    cout << " THEFINALCOUNTDOWN:::: " << quality_i << endl;

	    file_output << quality_i ;
		for(int i = 0 ; i < 10 ; i++)
	    	file_output << ' ' << i+1 << ":" << values_inline[i] ;
      	file_output << ' ' << endl;

    	count_line++;
    	quality_i = 0;	
	} // end for each line	

	file_output.close();

	cout << endl;
	cout << "Count rows : " << count_line << endl;
	cout << "quality_index : " << quality_index << endl;

	cout << " YEAH ENJOY " << endl;
	cout << "   fine   " << endl;

	ros::spin();
	return 0;
}