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
#include "normal_component_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;





//////////////////////////////////	BOX

std::vector<double> box;
//////////////////////////////////////////////


/////////////////////////////////// CONTACT POINTS

std::vector<double> contact_points;
///////////////////////////////////////


////////////////////////////////////  CONTACT FORCE

std::vector<double> contact_forces;
/////////////////////////////////////////////


////////////////////////////////////// JOINTS

std::vector<double> joints;
////////////////////////////////////////////


int n_c = 0;
int n_q = 0;


//////////////////////////////////////////// ROBOT SCTRUCTURE

KDL::Tree hand_tree("RootName");


	KDL::Chain chain_palm;
	KDL::Chain chain_finger_right;
	KDL::Chain chain_finger_left;

	KDL::Chain chain_palm_finger_left;
	KDL::Chain chain_palm_finger_right;

  KDL::JntArray jointpositions_right = JntArray(8);  
  KDL::JntArray jointpositions_left = JntArray(8);

	KDL::Jacobian jacobian_right;
	KDL::Jacobian jacobian_left;

	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_right;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_left;

///////////////////////////////////////////////////////////



//------------------------------------------------------------------------------------------
void createStructureKDL()
{
  	chain_palm.addSegment(Segment("palm_trasl_x_link",Joint(Joint::TransX),Frame(Vector(0.0,0.0,0.0))));	
   	chain_palm.addSegment(Segment("palm_trasl_y_link",Joint(Joint::TransY),Frame(Vector(0.0,0.0,0.0))));
	  chain_palm.addSegment(Segment("palm_trasl_z_link",Joint(Joint::TransZ),Frame(Vector(0.0,0.0,0.0))));
   	chain_palm.addSegment(Segment("palm_rot_x_link",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.0))));
   	chain_palm.addSegment(Segment("palm_rot_y_link",Joint(Joint::RotY),Frame(Vector(0.0,0.0,0.0))));
   	chain_palm.addSegment(Segment("palm_rot_z_link",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.0))));

	chain_finger_right.addSegment(Segment("finger_right_proximal_link",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.02))));
	chain_finger_right.addSegment(Segment("finger_right_distal_link",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.02))));

	chain_finger_left.addSegment(Segment("finger_left_proximal_link",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.02))));
	chain_finger_left.addSegment(Segment("finger_left_distal_link",Joint(Joint::RotZ),Frame(Vector(0.0,0.0,0.02))));
   	
	hand_tree.addChain(chain_palm,"RootName");   	
	hand_tree.addChain(chain_finger_right,"palm_rot_z_link");
	hand_tree.addChain(chain_finger_left, "palm_rot_z_link");	

	hand_tree.getChain("RootName", "finger_right_distal_link", chain_palm_finger_right);
	hand_tree.getChain("RootName", "finger_left_distal_link", chain_palm_finger_left);

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
void setJoints()
{
	for(int i=0 ; i < 6 ; i++)
   		jointpositions_right(i) = jointpositions_left(i) = joints[i];
 
  	jointpositions_right(6) = joints[6];
  	jointpositions_right(7) = joints[7];

  	jointpositions_left(6) = joints[8];
  	jointpositions_left(7) = joints[9];

	jointpositions_right.resize(chain_palm_finger_right.getNrOfJoints());
  	jointpositions_left.resize(chain_palm_finger_left.getNrOfJoints());
}



void initJacobian()
{
	jacobian_right.resize(chain_palm_finger_right.getNrOfJoints());
  	jacobian_left.resize(chain_palm_finger_left.getNrOfJoints());

  	jnt_to_jac_right.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_right));
  	jnt_to_jac_left.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_left));
}













int main (int argc, char **argv)
{

	ros::init(argc, argv, "Test_file");	
	ros::NodeHandle nh;




	nh.param<std::vector<double>>("box", box, std::vector<double>{1, 1, 1});
	nh.param<std::vector<double>>("contact_points", contact_points, std::vector<double>{1,1,1,1,1,1});
	nh.param<std::vector<double>>("contact_forces", contact_forces, std::vector<double>{1,1,1,1,1,1});
	nh.param<std::vector<double>>("joints", joints, std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
	nh.param<int>("n_c", n_c, 0);
	nh.param<int>("n_q", n_q, 0);


	if(n_c <= 0) 
		return 0;

	createStructureKDL();
	setJoints();
	initJacobian();

	
	Eigen::MatrixXd G_i = MatrixXd::Identity(6,6*n_c);
  Eigen::MatrixXd G_r = MatrixXd::Identity(6,6*n_c); 
  Eigen::MatrixXd Jacobian(6*n_c, n_q);
  	
  	int step  = 0;
  	int step_ = 0;
  	for ( int i=0; i < n_c ; i++)
  	{
		Eigen::MatrixXd Grasp(6,6);  
 		Eigen::MatrixXd Skew_Matrix(3,3);
  		Eigen::MatrixXd Rotation(3,3);
  		Eigen::MatrixXd b_Rotation_c(3,3);

  		Rotation = MatrixXd::Identity(3,3);
  		normal_component(b_Rotation_c, box[0]/2, box[1]/2, box[2]/2 , contact_points[step+0], contact_points[step+1], contact_points[step+2]);

  		Eigen::VectorXd f_c(3);
  		f_c(0) = contact_forces[step+0];
  		f_c(1) = contact_forces[step+1];
  		f_c(2) = contact_forces[step+2];

  		Eigen::VectorXd f_b = b_Rotation_c * f_c;

  		cout << " f_b " << endl;  cout << f_b << endl;  cout << endl;

  		Skew_Matrix(0,0) = Skew_Matrix(1,1) = Skew_Matrix(2,2) = 0;
     	Skew_Matrix(0,1) = - contact_points[step+2]; // -rz    
     	Skew_Matrix(0,2) = contact_points[step+1];   // ry
        Skew_Matrix(1,0) = contact_points[step+2];   // rz
        Skew_Matrix(2,0) = - contact_points[step+1]; // -ry
        Skew_Matrix(1,2) = - contact_points[step+0]; // -rx
       	Skew_Matrix(2,1) = contact_points[step+0];   // rx

		Grasp.block<3,3>(0,0) = Rotation;
  		Grasp.block<3,3>(3,3) = Rotation;
  		Grasp.block<3,3>(3,0) = Skew_Matrix * Rotation;
   		Grasp.block<3,3>(0,3) = MatrixXd::Zero(3,3);

   		G_i.block<6,6>(0,step_) = Grasp;
       			
   		Grasp.block<3,3>(0,0) = b_Rotation_c;
  		Grasp.block<3,3>(3,3) = b_Rotation_c;
  		Grasp.block<3,3>(3,0) = Skew_Matrix * b_Rotation_c;
   		Grasp.block<3,3>(0,3) = MatrixXd::Zero(3,3);

   		G_r.block<6,6>(0,step_) = Grasp;
   		
   		jnt_to_jac_right->JntToJac(jointpositions_right, jacobian_right, -1);
  		jnt_to_jac_left->JntToJac(jointpositions_left, jacobian_left, -1);		
   		if( !std::isnan(contact_points[0]))
   		{
   			if( !std::isnan(contact_points[3]))
   			{	
   				jnt_to_jac_right->JntToJac(jointpositions_right, jacobian_right);
  				jnt_to_jac_left->JntToJac(jointpositions_left, jacobian_left);
  			}
  			else
  				jnt_to_jac_left->JntToJac(jointpositions_left, jacobian_left); 			
  		}
  		else
  			if( !std::isnan(contact_points[3])) 
  				jnt_to_jac_right->JntToJac(jointpositions_right, jacobian_right);

		Jacobian.block<6,8>(step_,0) = jacobian_right.data;
  		Jacobian.block<6,2>(step_,8) = jacobian_left.data.block<6,2>(0,6);

  		step += 3;
  		step_ += 6;
  	}

  	
  	Eigen::VectorXd w_r(6);
  	Eigen::VectorXd w_i(6);
  	Eigen::VectorXd f(3*n_c);
  	Eigen::MatrixXd H(3*n_c,6*n_c);
  	Eigen::MatrixXd H_(3,6);

  	H_ << 1, 0, 0, 0, 0, 0,
		  0, 1, 0, 0, 0, 0,
		  0, 0, 1, 0, 0, 0;

	int i = 0;
	int j = 0;
	for(int n = 0 ;  n < n_c ; n++)
	{
		H.block<3,6>(i,j) = H_;
		i += 3;
		j += 6;
	}

	Eigen::MatrixXd G_ht_i = G_i*H.transpose();
	Eigen::MatrixXd G_ht_r = G_r*H.transpose();


	for(int i = 0 ; i < 6 ; i++)
	  	f(i) = contact_forces[i];


  	w_i = - G_ht_i * f;
  	w_r = - G_ht_r * f;

  	cout << "f : " << endl;	cout << f << endl;  cout << endl;
  	//cout << " G_i : " << endl; cout << G_i << endl; cout << endl;
  	//cout << " G_f : " << endl; cout << G_r << endl; cout << endl;
  	cout << " w_i : " << endl; cout << w_i << endl; cout << endl;
  	cout << " w_r : " << endl; cout << w_r << endl; cout << endl;

	ros::spinOnce();
	return 0;
}