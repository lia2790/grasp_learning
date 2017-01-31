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


#include "pseudo_inverse.h"
#include "quality.h"
#include "quality_PCR_PGR.h"




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
double quality_i = 9999;




int main (int argc, char **argv)
{

	ros::init(argc, argv, "Grasp_quality");	// ROS node
	ros::NodeHandle nh;


 	
	nh.param<int>("n_rows_file",n_rows,108);
	nh.param<int>("n_cols_file",n_cols,86);
	nh.param<int>("quality_index",quality_index,0);
	nh.param<std::string>("file_name", relative_path_file, "/db/box_db_2.csv" );
	nh.param<std::string>("frame_name_thumb", frame_name_finger[0], "right_hand_thumb_distal_link");
	nh.param<std::string>("frame_name_index", frame_name_finger[1], "right_hand_index_distal_link");
	nh.param<std::string>("frame_name_middle", frame_name_finger[2], "right_hand_middle_distal_link");
	nh.param<std::string>("frame_name_ring", frame_name_finger[3], "right_hand_ring_distal_link");
	nh.param<std::string>("frame_name_little", frame_name_finger[4], "right_hand_little_distal_link");
	nh.param<std::string>("frame_name_root", frame_name_root, "world");
	nh.param<int>("number_of_fingers", n_fingers, 5);



	

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
	}











	///////////////////// get values from file for each line //////////////////////////////////

	int first_element_joint_array = 17;
	int number_of_joints = 19;
	int number_of_contact_point = 16;
	int first_element_cp_array = number_of_joints + first_element_joint_array;


	while( !file.eof() )
	{
		for(std::string line; getline( file, line, '\n' ); ) // for each line
		{
			std::vector<double> values_inline;
    		std::istringstream iss_line(line);	
    		for(std::string value; getline(iss_line, value, ',' ); )
    			values_inline.push_back(stod(value));

    		for (int i = 0 ; i < values_inline.size(); i++)
    			cout << " line " << i << " : " << values_inline[i] << endl;
    		
    	
    		KDL::Vector trasl_w_T_o(0,0, -(values_inline[2]/2)); // i m not sure if it is corrected
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


    			if(i == 0) // thumb
    			{
    				q_finger[i](6) = values_inline[first_element_joint_array];

					q_finger[i](7) = values_inline[first_element_joint_array+1] / 2;
					q_finger[i](8) = values_inline[first_element_joint_array+1] / 2;

					q_finger[i](9) = values_inline[first_element_joint_array+2] / 2;
					q_finger[i](10) = values_inline[first_element_joint_array+2] / 2;

					k_+=3;
				}
				else
				{
					q_finger[i](6) = values_inline[first_element_joint_array+k_];

					q_finger[i](7) = values_inline[first_element_joint_array+k_+1] / 2;
					q_finger[i](8) = values_inline[first_element_joint_array+k_+1] / 2;

					q_finger[i](9) = values_inline[first_element_joint_array+k_+2] / 2;
					q_finger[i](10) = values_inline[first_element_joint_array+k_+2] / 2;

					q_finger[i](11) = values_inline[first_element_joint_array+k_+3] / 2;
					q_finger[i](12) = values_inline[first_element_joint_array+k_+3] / 2;

					k_+=4;
				}			
			}


			Eigen::MatrixXd Contacts(number_of_contact_point,3);
			int gap_coordinate = 0;

			for(int i = 0; i < number_of_contact_point; i++)
			{	
				for(int j = 0; j < 3; j++ )
					Contacts(i,j) = values_inline[first_element_cp_array + gap_coordinate + j];  // check if it is correct

				gap_coordinate += 3;
			}


			cout << "Contacts : " << Contacts << endl;




			string not_a_n = "nan";

		
    		//for each contact point 
			std::vector<Eigen::MatrixXd> Grasp_Matrix_ ;  // G
    		std::vector<Eigen::MatrixXd> Hand_Jacobian_ ; // J


    		int k = 1;

			for(int i = 0 ; i < number_of_contact_point ; i++) //calc the grasp_matrix and hand_jacobian for each contact point
    		{

      			if( !std::isnan(Contacts(i,0)) ) // NaN in dataset
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
        			Grasp_Matrix.block<3,3>(0,3) = Skew_Matrix * Rotation;
        			Grasp_Matrix.block<3,3>(3,0) = MatrixXd::Zero(3,3);


        			cout << "Grasp_Matrix" << endl;
        			cout << Grasp_Matrix << endl;


       				Grasp_Matrix_.push_back(Grasp_Matrix);


       				for(int n = 0 ; n < n_fingers ; n++)
        				jnt_to_jac_solver[n]->JntToJac(q_finger[n], hand_jacob[n]);




   					cout << "Hand_Jacobian_ THUMB" << endl;
      				cout <<  hand_jacob[0].data << endl;
      				cout << "Hand_Jacobian_ INDEX" << endl;
  		  			cout <<  hand_jacob[1].data << endl;
  					cout << "Hand_Jacobian_ MIDDLE"<< endl;
   					cout <<  hand_jacob[2].data << endl;
      				cout << "Hand_Jacobian_ RING"  << endl;
    				cout <<  hand_jacob[3].data << endl;
  					cout << "Hand_Jacobian_ LITTLE"<< endl;
  					cout <<  hand_jacob[4].data << endl;



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
	            

	            	jnt_to_jac_solver[which_finger]->JntToJac(q_finger[which_finger],hand_jacob[which_finger], which_falange);


        		
	  				cout << " Jacobian thumb " << hand_jacob[0].data << endl ;
	  				cout << " Jacobian index " << hand_jacob[1].data << endl ;
	  				cout << " Jacobian middle " << hand_jacob[2].data << endl ;
					cout << " Jacobian ring " << hand_jacob[3].data << endl ;
					cout << " Jacobian little " << hand_jacob[4].data << endl ;



				  	Hand_Jacobian_.push_back(hand_jacob[0].data); // 5
				  	Hand_Jacobian_.push_back(hand_jacob[1].data); // 7
				 	Hand_Jacobian_.push_back(hand_jacob[2].data); // 7
			  		Hand_Jacobian_.push_back(hand_jacob[3].data); // 7
	  				Hand_Jacobian_.push_back(hand_jacob[4].data); // 7
	      		}// end if  contact
        	} // end for each contact point	



      		cout << " Dim of the vectors of the matrices grasp " << Grasp_Matrix_.size() << endl;
    		cout << " Dim of the hand jacobian " << Hand_Jacobian_.size() << endl;


    		int n_c_eff = Grasp_Matrix_.size();
    		int nq_hand = hand_tree.getNrOfJoints();
      
    		if(n_c_eff != 0 ) // i have a cp
    		{
    	
      			Eigen::MatrixXd Grasp_Matrix_Contact(6,6*n_c_eff); 		  // G 6x6N_c
    			Eigen::MatrixXd Hand_Jacobian_Contact(6*n_c_eff,nq_hand-1); // J 6N_cxN_q
      

				int s = 0; 
				int s_ = 0;


      			for(int i = 0; i < n_c_eff; i++)
      			{

        			Grasp_Matrix_Contact.block<6,6>(0,s) = Grasp_Matrix_[i];
        	
        			cout << "i : " << i << endl;

      				Hand_Jacobian_Contact.block<6,5>(s,0) = Hand_Jacobian_[s_];
      				Hand_Jacobian_Contact.block<6,7>(s,5) = Hand_Jacobian_[s_+1];
      				Hand_Jacobian_Contact.block<6,7>(s,12) = Hand_Jacobian_[s_+2];
      				Hand_Jacobian_Contact.block<6,7>(s,19) = Hand_Jacobian_[s_+3];
      				Hand_Jacobian_Contact.block<6,7>(s,26) = Hand_Jacobian_[s_+4];
			
  					s_+=5;
       				s+=6;
      			}

				/*file_output << "Grasp_Matrix_Contact : " << endl;
       		 	file_output <<  Grasp_Matrix_Contact << endl;
        		file_output << "__________________________________________________" << endl;
        		file_output << "Hand_Jacobian_Contact : " << endl;
        		file_output << Hand_Jacobian_Contact << endl;*/

    			quality_i = quality(quality_index, Grasp_Matrix_Contact, Hand_Jacobian_Contact); 
      		}
      		else
      		{
      			Eigen::MatrixXd Grasp_Matrix_Contact(6,6); 		  // G 6x6n_c
    			Eigen::MatrixXd Hand_Jacobian_Contact(6,6); 	  // J 6n_cxn_q

    			Grasp_Matrix_Contact  <<  MatrixXd::Identity(6,6); 
      			Hand_Jacobian_Contact <<  MatrixXd::Identity(6,6); 

      			quality_i = quality(-1, Grasp_Matrix_Contact, Hand_Jacobian_Contact);
      		}


	    	file_output << quality_i ;

			for(int i = 0 ; i < 10 ; i++)
	      		file_output << ' ' << i+1 << ":" << values_inline[i] ;

	      	file_output << ' ' << endl;

    	  	
		} // end for each line
	}// end of file







	file_output.close();




	cout << " YEAH ENJOY " << endl;
	cout << "   fine   " << endl;


	ros::spin();
	return 0;
}