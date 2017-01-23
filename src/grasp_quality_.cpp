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


int n_rows = 0;
int n_cols = 0;


int dim_file_row = 108;
int dim_file_col = 10; 


int n_c = 19; // numero di punti di contatto avendo supposto di avere 
			  // un punto di contatto per ogni falange 
			  // 3 falangi per il pollice
			  // 4 per le altre dita  



int quality_index = 0;



std::vector<double> Quality;



int main (int argc, char **argv)
{

	ros::init(argc, argv, "Soft_Hand_Jacobian");	// ROS node
	ros::NodeHandle nh;


////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

//	TAKE DATA_BASE : load a file .csv and put the values into Eigen::MatrixXd

////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////



std::string file_name;

if(argc > 1)
	file_name = std::string( argv[1] );
else
	file_name = std::string("box_db.csv");




ifstream file(file_name); 



/*

cout << "3:::::::::::::::" << endl;


bool count_cols_only_one = true;


	for(std::string line; getline ( file, line, '\n' ); ) // ciclo sulla riga
	{
    	cout << "Value " << line << endl;

    	n_rows++;

    	std::istringstream iss_line(line);	

    	

    	if(count_cols_only_one)
    	{	

    		for(std::string value; getline(iss_line, value, ',' ); )
    		{
    			cout << "Value : " << value << " Value stod : " << stod(value) << endl;
    			n_cols++;
    		}


    		count_cols_only_one = false;
    	}
    	

    	
	}

	cout << "ROWS : " << n_rows << endl;
	cout << "COLS : " << n_cols << endl;


	cout << "7:::::::::::" << endl;

*/


	if(	(n_rows == 0) || (n_cols==0) )
	{	
		n_rows = dim_file_row;
		n_cols = dim_file_col;
	}



	Eigen::MatrixXd data_set(n_rows,n_cols);


	int i = 0;
	int j = 0;



	for(std::string line; getline( file, line, '\n' ); ) // ciclo sulla riga
	{

    	std::istringstream iss_line(line);	

  
    	for(std::string value; getline(iss_line, value, ',' ); )
    	{
    		data_set(i,j) = stod(value);
    		j++;
    	}

    	cout << endl;

    	j=0;
    	i++;
	}


	file.close();





	cout << " DATA_SET" << data_set << endl;



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



  KDL::Chain hand_finger[5];
  KDL::Jacobian hand_jacob[5];




  std::string root_name = "right_hand_palm_link" ;
  std::string end_chain_name[5];


  end_chain_name[0] = "right_hand_thumb_distal_link";
  end_chain_name[1] = "right_hand_index_distal_link";
  end_chain_name[2] = "right_hand_middle_distal_link";
  end_chain_name[3] = "right_hand_ring_distal_link";
  end_chain_name[4] = "right_hand_little_distal_link";


  hand_tree.getChain(root_name, end_chain_name[0], hand_finger[0]);      
  hand_tree.getChain(root_name, end_chain_name[1], hand_finger[1]); 
  hand_tree.getChain(root_name, end_chain_name[2], hand_finger[2]);
  hand_tree.getChain(root_name, end_chain_name[3], hand_finger[3]);
  hand_tree.getChain(root_name, end_chain_name[4], hand_finger[4]);


 



///////////////////////////// 		END		///////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

		
		
////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

//			CALCULATE THE GRASP_MATRIX AND HAND_JACOBIAN FOR EACH ROW

//	I supposed to have 19 joint variables and a single point of contact for each phalanx

////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////



		unsigned int nj_0 = hand_finger[0].getNrOfJoints();   // 5
  		unsigned int nj_1 = hand_finger[1].getNrOfJoints();   // 7
  		unsigned int nj_2 = hand_finger[2].getNrOfJoints();  // 7
  		unsigned int nj_3 = hand_finger[3].getNrOfJoints();    // 7
  		unsigned int nj_4 = hand_finger[4].getNrOfJoints();  // 7 




  		KDL::JntArray q_thumb = JntArray(nj_0);    // thumb
    	KDL::JntArray q_index = JntArray(nj_1);    // forefinger
    	KDL::JntArray q_middle = JntArray(nj_2);    // middlefinger
    	KDL::JntArray q_ring = JntArray(nj_3);    // ringfinger
    	KDL::JntArray q_little = JntArray(nj_4);    // littlefinger





  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_0;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_1;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_2;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_3;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_4;


  // constructs the kdl solvers in non-realtime
  jnt_to_jac_solver_0.reset(new KDL::ChainJntToJacSolver(hand_finger[0]));
  jnt_to_jac_solver_1.reset(new KDL::ChainJntToJacSolver(hand_finger[1]));
  jnt_to_jac_solver_2.reset(new KDL::ChainJntToJacSolver(hand_finger[2]));
  jnt_to_jac_solver_3.reset(new KDL::ChainJntToJacSolver(hand_finger[3]));
  jnt_to_jac_solver_4.reset(new KDL::ChainJntToJacSolver(hand_finger[4]));





   		


    	// resizes the joint state vectors in non-realtime
  		q_thumb.resize(hand_finger[0].getNrOfJoints());
  		q_index.resize(hand_finger[1].getNrOfJoints());
  		q_middle.resize(hand_finger[2].getNrOfJoints());
  		q_ring.resize(hand_finger[3].getNrOfJoints());
  		q_little.resize(hand_finger[4].getNrOfJoints());

  		hand_jacob[0].resize(hand_finger[0].getNrOfJoints());
  		hand_jacob[1].resize(hand_finger[1].getNrOfJoints());
  		hand_jacob[2].resize(hand_finger[2].getNrOfJoints());
 		hand_jacob[3].resize(hand_finger[3].getNrOfJoints());
  		hand_jacob[4].resize(hand_finger[4].getNrOfJoints());







	for(int r = 0; r < n_rows ; r++)  // for each row calculation the grasp matrix and hand jacobian
	{
		//take the value of joints from data		
		// initialize jntarray then be able to calculate the Jacobian to that point of contact


		q_thumb(0) = data_set(r,10);

		q_thumb(1) = data_set(r,11) / 2;
		q_thumb(2) = data_set(r,11) / 2;

		q_thumb(3) = data_set(r,12) / 2;
		q_thumb(4) = data_set(r,12) / 2;



		q_index(0) = data_set(r,13);

		q_index(1) = data_set(r,14) / 2;
		q_index(2) = data_set(r,14) / 2;

		q_index(3) = data_set(r,15) / 2;
		q_index(4) = data_set(r,15) / 2;

		q_index(5) = data_set(r,16) / 2;
		q_index(6) = data_set(r,16) / 2;



		q_middle(0) = data_set(r,17);

		q_middle(1) = data_set(r,18) / 2;
		q_middle(2) = data_set(r,18) / 2;

		q_middle(3) = data_set(r,19) / 2;
		q_middle(4) = data_set(r,19) / 2;

		q_middle(5) = data_set(r,20) / 2;
		q_middle(6) = data_set(r,20) / 2;



		q_ring(0) = data_set(r,21);

		q_ring(1) = data_set(r,22) / 2;
		q_ring(2) = data_set(r,22) / 2;

		q_ring(3) = data_set(r,23) / 2;
		q_ring(4) = data_set(r,23) / 2;

		q_ring(5) = data_set(r,24) / 2;
		q_ring(6) = data_set(r,24) / 2;





		q_little(0) = data_set(r,25);

		q_little(1) = data_set(r,26) / 2;
		q_little(2) = data_set(r,26) / 2;

		q_little(3) = data_set(r,27) / 2;
		q_little(4) = data_set(r,27) / 2;

		q_little(5) = data_set(r,28) / 2;
		q_little(6) = data_set(r,28) / 2;










		Eigen::MatrixXd Contacts(n_c,3); // I build a matrix of contact points for convenience only
										 // each row is a contact point
										 // rows 0 and 1 for thumb
										 // rows 2, 3, 4 for index
										 // rows 5, 6, 7 for middle
										 // rows 8, 9, 10 for ring
										 // rows 11, 12, 13 for little



		
		int start = 29; // indice del primo valore utile nel database 
		int gap_point_coordinate = 0;

		for(int k = 0; k < n_c; k++)
		{	
			for(int h = 0; h < 3; h++ )
				Contacts(k,h) = data_set(i, h + start + gap_point_coordinate);

			gap_point_coordinate += 3;

		}


		












		int which_finger = 0;

      	//for each contact point 
		std::vector<Eigen::MatrixXd> Grasp_Matrix_ ;  //
      	std::vector<Eigen::MatrixXd> Hand_Jacobian_ ;



      	for(int i = 0 ; i < n_c ; i++) //calc the grasp_matrix and hand_jacobian for each contact point
      	{
      		
      		if(Contacts(i,0) != 99999) //99999 similar to NaN in dataset
      		{	


      			Eigen::MatrixXd Grasp_Matrix(6,6);
 				Eigen::MatrixXd Skew_Matrix(3,3);
  				Eigen::MatrixXd Rotation(3,3);

  				Rotation <<  MatrixXd::Identity(3,3); 




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


            	Grasp_Matrix_.push_back(Grasp_Matrix);



            	jnt_to_jac_solver_0->JntToJac(q_thumb, hand_jacob[0], 0);
      			jnt_to_jac_solver_1->JntToJac(q_index, hand_jacob[1], 0);
      			jnt_to_jac_solver_2->JntToJac(q_middle, hand_jacob[2], 0);
      			jnt_to_jac_solver_3->JntToJac(q_ring, hand_jacob[3], 0);
      			jnt_to_jac_solver_4->JntToJac(q_little, hand_jacob[4], 0);








            	int which_falange = 0;


            	if((0 <= i) && (i <= 2)){ which_finger = 0; which_falange = i;}
            	if((3 <= i) && (i <= 6)){ which_finger = 1; which_falange = i-3;}
            	if((7 <= i) && (i <= 10)){ which_finger = 2; which_falange = i-7;}
            	if((11 <= i) && (i <= 14)){ which_finger = 3; which_falange = i-11;}
            	if((15 <= i) && (i <= 18)){ which_finger = 4; which_falange = i-15;}


            switch(which_finger)
            {
            	case 0: // thumb
      						jnt_to_jac_solver_0->JntToJac(q_thumb, hand_jacob[0], which_falange);
      						break;

	  			case 1: // index
      						jnt_to_jac_solver_1->JntToJac(q_index, hand_jacob[1], which_falange);
      						break;

	  			case 2: // middle
							jnt_to_jac_solver_2->JntToJac(q_middle, hand_jacob[2], which_falange);
      						break;

	  			case 3: // ring
      						jnt_to_jac_solver_3->JntToJac(q_ring, hand_jacob[3], which_falange);
	  						break;

	  			case 4: // little
      						jnt_to_jac_solver_4->JntToJac(q_little, hand_jacob[4], which_falange);
	  						break;

	  		}// end switch

	  		Hand_Jacobian_.push_back(hand_jacob[0].data);
	  		Hand_Jacobian_.push_back(hand_jacob[1].data);
	 		Hand_Jacobian_.push_back(hand_jacob[2].data);
	  		Hand_Jacobian_.push_back(hand_jacob[3].data);
	  		Hand_Jacobian_.push_back(hand_jacob[4].data);




        	}// and if true contact
        	



        }// end for contact










        int quality_ = 9999;



        switch(quality_index)
        {
        	case 0: // "minimum_singular_value_of_G"


					for(int k = 0; k < Grasp_Matrix_.size(); k++ )
					{	
						JacobiSVD<MatrixXd> svd(Grasp_Matrix_[k], ComputeThinU | ComputeThinV);  

						Eigen::VectorXd Singular_Value_Grasp_Matrix;
						Singular_Value_Grasp_Matrix = svd.singularValues();


						double sigma_min = Singular_Value_Grasp_Matrix[Singular_Value_Grasp_Matrix.size()];

						if(sigma_min < quality_) quality_ = sigma_min;

					}

					Quality.push_back(quality_);

        		break;







        	case 1: // "Grasp isotropy index"



					for(int k = 0; k < Grasp_Matrix_.size(); k++ )
					{	
						JacobiSVD<MatrixXd> svd(Grasp_Matrix_[k], ComputeThinU | ComputeThinV);  

						Eigen::VectorXd Singular_Value_Grasp_Matrix;
						Singular_Value_Grasp_Matrix = svd.singularValues();


						double sigma_min = Singular_Value_Grasp_Matrix[Singular_Value_Grasp_Matrix.size()];
						double sigma_max = Singular_Value_Grasp_Matrix[0];


						double quality_in = sigma_min / sigma_max ;

						if(quality_in < quality_) quality_ = quality_in;

					}

					Quality.push_back(quality_);

        		break;




        }










      	



	} // per ogni riga !!!!!!!!!!! panic !!!!!!!!!!!!!









  file.close();

  cout << " YEAH ENJOY " << endl;
  cout << "   fine   " << endl;


	ros::spin();
	return 0;

}