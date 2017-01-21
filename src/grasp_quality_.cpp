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


#include <cmath>
#include <ctime>
#include <time.h>




using namespace std;
using namespace Eigen;
using namespace KDL;


int n_rows = 1;
int n_cols = 1;


int main (int argc, char **argv)
{

	ros::init(argc, argv, "Soft_Hand_Jacobian");	// ROS node
	ros::NodeHandle nh;


////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

//									TAKE DATA_BASE

////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////





	std::fstream file("box_db.csv"); // load the file.csv
	std::string line;

	bool is_count = true; // flag

	while(std::getline(file,line))  // loop for count the dimension of data base 
	{								// and then build an array containing all value 
		n_rows++;

		char *vettore;
		vettore = (char *)line.c_str();


		if(is_count)
		{	
			for(int j = 0 ; j < line.length() ; j++)
			{
				if(vettore[j] == ' ') break;
				if(vettore[j] == ',') break;

				n_cols++;
			}
			
			is_count = false;
		}

	}


	cout << "rows_in_data_set : " << n_rows << endl;
	cout << "cols_in_data_set : " << n_cols << endl;




	Eigen::MatrixXd data_set(n_rows,n_cols);


	int i = 0;
	int j = 0;

	


	while(std::getline(file,line))	// take the value from file
	{

		char *vettore;
		vettore = (char *)line.c_str();

		for(int k = 0 ; k < line.length() ; k++)
		{
			if(vettore[k] == ' ') break;
			if(vettore[k] == ',') break;

			data_set(i,j) = (double)vettore[k];
			j++;
		}	

		j=0;
		i++;
	}




///////////////////////////// 		END		///////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

		



////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

//								TAKE SOFT_HAND

////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////







  KDL::Tree hand_tree;

  std::string robot_desc_string;
  nh.param("robot_description", robot_desc_string, string());  // robot description is the name in the launch file 
  if (!kdl_parser::treeFromString(robot_desc_string, hand_tree))
  {     ROS_ERROR("Failed to construct kdl tree");
        return false;       }




  int nj_hand = hand_tree.getNrOfJoints();  // 34
  int ns_hand = hand_tree.getNrOfSegments(); // 39


  cout << "number_of_joint_in_hand : " << nj_hand << endl;
  cout << "number_of_segment_in_hand : " << ns_hand << endl;



  KDL::Chain chain0_thumb;
  KDL::Chain chain1_index;
  KDL::Chain chain2_middle;
  KDL::Chain chain3_ring;
  KDL::Chain chain4_little;




  std::string root_name = "right_hand_palm_link" ;
  std::string end_chain_name[5] ;


  end_chain_name[0] = "right_hand_thumb_distal_link";
  end_chain_name[1] = "right_hand_index_distal_link";
  end_chain_name[2] = "right_hand_middle_distal_link";
  end_chain_name[3] = "right_hand_ring_distal_link";
  end_chain_name[4] = "right_hand_little_distal_link";


  hand_tree.getChain(root_name, end_chain_name[0], chain0_thumb);      
  hand_tree.getChain(root_name, end_chain_name[1], chain1_index); 
  hand_tree.getChain(root_name, end_chain_name[2], chain2_middle);
  hand_tree.getChain(root_name, end_chain_name[3], chain3_ring);
  hand_tree.getChain(root_name, end_chain_name[4], chain4_little);


 



///////////////////////////// 		END		///////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

		
		
////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

//			CALCULATE THE GRASP_MATRIX AND HAND_JACOBIAN FOR EACH ROW

//	I supposed to have 19 joint variables and a single point of contact for each phalanx

////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////


  		double q_thumb[5] ;
		double q_index[7] ;
		double q_middle[7] ;
		double q_ring[7] ;
		double q_little[7] ;

		KDL::Jacobian jacobian0_thumb;
		KDL::Jacobian jacobian1_index;
		KDL::Jacobian jacobian2_middle;
		KDL::Jacobian jacobian3_ring;
		KDL::Jacobian jacobian4_little;


		unsigned int nj_0 = chain0_thumb.getNrOfJoints();   // 5
  		unsigned int nj_1 = chain1_index.getNrOfJoints();   // 7
  		unsigned int nj_2 = chain2_middle.getNrOfJoints();  // 7
  		unsigned int nj_3 = chain3_ring.getNrOfJoints();    // 7
  		unsigned int nj_4 = chain4_little.getNrOfJoints();  // 7 



  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_0;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_1;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_2;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_3;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_4;


  // constructs the kdl solvers in non-realtime
  jnt_to_jac_solver_0.reset(new KDL::ChainJntToJacSolver(chain0_thumb));
  jnt_to_jac_solver_1.reset(new KDL::ChainJntToJacSolver(chain1_index));
  jnt_to_jac_solver_2.reset(new KDL::ChainJntToJacSolver(chain2_middle));
  jnt_to_jac_solver_3.reset(new KDL::ChainJntToJacSolver(chain3_ring));
  jnt_to_jac_solver_4.reset(new KDL::ChainJntToJacSolver(chain4_little));





   		KDL::JntArray jointpositions_0 = JntArray(nj_0);    // thumb
    	KDL::JntArray jointpositions_1 = JntArray(nj_1);    // forefinger
    	KDL::JntArray jointpositions_2 = JntArray(nj_2);    // middlefinger
    	KDL::JntArray jointpositions_3 = JntArray(nj_3);    // ringfinger
    	KDL::JntArray jointpositions_4 = JntArray(nj_4);    // littlefinger



    	// resizes the joint state vectors in non-realtime
  		jointpositions_0.resize(chain0_thumb.getNrOfJoints());
  		jointpositions_1.resize(chain1_index.getNrOfJoints());
  		jointpositions_2.resize(chain2_middle.getNrOfJoints());
  		jointpositions_3.resize(chain3_ring.getNrOfJoints());
  		jointpositions_4.resize(chain4_little.getNrOfJoints());

  		jacobian0_thumb.resize(chain0_thumb.getNrOfJoints());
  		jacobian1_index.resize(chain1_index.getNrOfJoints());
  		jacobian2_middle.resize(chain2_middle.getNrOfJoints());
 		jacobian3_ring.resize(chain3_ring.getNrOfJoints());
  		jacobian4_little.resize(chain4_little.getNrOfJoints());







	for(int i = 0; i < n_rows ; i++)  // for each row calculation the grasp matrix and hand jacobian
	{
		//take the value of joints from data

		q_thumb[0] = data_set(i,10);

		q_thumb[1] = data_set(i,11) / 2;
		q_thumb[2] = data_set(i,11) / 2;

		q_thumb[3] = data_set(i,12) / 2;
		q_thumb[4] = data_set(i,12) / 2;



		q_index[0] = data_set(i,13);

		q_index[1] = data_set(i,14) / 2;
		q_index[2] = data_set(i,14) / 2;

		q_index[3] = data_set(i,15) / 2;
		q_index[4] = data_set(i,15) / 2;

		q_index[5] = data_set(i,16) / 2;
		q_index[6] = data_set(i,16) / 2;



		q_middle[0] = data_set(i,17);

		q_middle[1] = data_set(i,18) / 2;
		q_middle[2] = data_set(i,18) / 2;

		q_middle[3] = data_set(i,19) / 2;
		q_middle[4] = data_set(i,19) / 2;

		q_middle[5] = data_set(i,20) / 2;
		q_middle[6] = data_set(i,20) / 2;



		q_ring[0] = data_set(i,21);

		q_ring[1] = data_set(i,22) / 2;
		q_ring[2] = data_set(i,22) / 2;

		q_ring[3] = data_set(i,23) / 2;
		q_ring[4] = data_set(i,23) / 2;

		q_ring[5] = data_set(i,24) / 2;
		q_ring[6] = data_set(i,24) / 2;





		q_little[0] = data_set(i,25);

		q_little[1] = data_set(i,26) / 2;
		q_little[2] = data_set(i,26) / 2;

		q_little[3] = data_set(i,27) / 2;
		q_little[4] = data_set(i,27) / 2;

		q_little[5] = data_set(i,28) / 2;
		q_little[6] = data_set(i,28) / 2;



		// initialize jntarray then be able to calculate the Jacobian to that point of contact

		for ( int i = 0 ; i < 5 ; i++)
          	jointpositions_0(i) = q_thumb[i];


	    for ( int i = 0 ; i < 7 ; i++)
      	{    
        	jointpositions_1(i) = q_index[i];
          	jointpositions_2(i) = q_middle[i];
          	jointpositions_3(i) = q_ring[i];
          	jointpositions_4(i) = q_little[i];
      	}   








		Eigen::MatrixXd Contacts(14,3);  // I build a matrix of contact points for convenience only


		int gap_point_coordinate = 0;

		for(int k = 0; k < 15; k++)
		{	
			for(int h = 0; h < 3; h++ )
				Contacts(k,h) = data_set(i,29+h+gap_point_coordinate);

			gap_point_coordinate += 3;

		}











      	int count = 0;


      	std::vector<Eigen::MatrixXd> Grasp_Matrix_ ;
      	//std::vector<std::vector<Eigen::MatrixXd>> Hand_Jacobian_ ;

      	for(int r = 0 ; r < 15 ; r++)
      	{
      		
      		if(Contacts(r,0) != 99999)
      		{	


      			Eigen::MatrixXd Grasp_Matrix(6,6);
 				Eigen::MatrixXd Skew_Matrix(3,3);
  				Eigen::MatrixXd Rotation(3,3);

  				Rotation <<  MatrixXd::Identity(3,3); 




      		Skew_Matrix(0,0) = Skew_Matrix(1,1) = Skew_Matrix(2,2) = 0;
            Skew_Matrix(0,1) = - Contacts(r,2); // -rz
            Skew_Matrix(0,2) = Contacts(r,1);   // ry
            Skew_Matrix(1,0) = Contacts(r,2);   // rz
            Skew_Matrix(2,0) = - Contacts(r,1); // -ry
            Skew_Matrix(1,2) = - Contacts(r,0); // -rx
            Skew_Matrix(2,1) = Contacts(r,0);   // rx



            Grasp_Matrix.block<3,3>(0,0) = Rotation;
            Grasp_Matrix.block<3,3>(3,3) = Rotation;
            Grasp_Matrix.block<3,3>(0,3) = Skew_Matrix * Rotation;
            Grasp_Matrix.block<3,3>(3,0) = MatrixXd::Zero(3,3);


            Grasp_Matrix_.push_back(Grasp_Matrix);



            // compute Jacobian in realtime
      jnt_to_jac_solver_0->JntToJac(jointpositions_0, jacobian0_thumb);
      jnt_to_jac_solver_1->JntToJac(jointpositions_1, jacobian1_index);
      jnt_to_jac_solver_2->JntToJac(jointpositions_2, jacobian2_middle);
      jnt_to_jac_solver_3->JntToJac(jointpositions_3, jacobian3_ring);
      jnt_to_jac_solver_4->JntToJac(jointpositions_4, jacobian4_little);


      std::vector<Eigen::MatrixXd>  Jacobian_finger;// KDL::Jacobian jacobian0_thumb;
                                                    // KDL::Jacobian jacobian1_forefinger;
                                                    // KDL::Jacobian jacobian2_middlefinger;
                                                    // KDL::Jacobian jacobian3_ringfinger;
                                                    // KDL::Jacobian jacobian4_littlefinger;



      Jacobian_finger.push_back(jacobian0_thumb.data);
      Jacobian_finger.push_back(jacobian1_index.data);
      Jacobian_finger.push_back(jacobian2_middle.data);
      Jacobian_finger.push_back(jacobian3_ring.data);
      Jacobian_finger.push_back(jacobian4_little.data);



        	}
        }




















      	



	} // per ogni riga !!!!!!!!!!! panic !!!!!!!!!!!!!

  file.close();

}