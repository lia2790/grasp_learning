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
#include "PGR_5.h"
#include "normal_component_box_surface.h"


using namespace std;
using namespace Eigen;
using namespace KDL;
const double PPI_  = 3.141592653589793238463;

const double PI_  = 3.141592653589793238463;

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

Eigen::VectorXd joints(10);
////////////////////////////////////////////


int n_c = 0;
int n_q = 0;
double quality_i = 0;
double contact_stiffness = 100;
double joint_stiffness = 1; 
double mu = 1.5;
double f_i_max = 200;


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

	chain_finger_right.addSegment(Segment("finger_right_proximal_link",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
	chain_finger_right.addSegment(Segment("finger_right_distal_link",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

	chain_finger_left.addSegment(Segment("finger_left_proximal_link",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.3))));
	chain_finger_left.addSegment(Segment("finger_left_distal_link",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
   	
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

  std::vector<double> joints_;


	nh.param<std::vector<double>>("box", box, std::vector<double>{1, 1, 1});
	nh.param<std::vector<double>>("contact_points", contact_points, std::vector<double>{1,1,1,1,1,1});
	nh.param<std::vector<double>>("contact_forces", contact_forces, std::vector<double>{1,1,1,1,1,1});
	nh.param<std::vector<double>>("joints", joints_, std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  nh.param<double>("mu", mu, 0.5);
  nh.param<double>("f_i_max", f_i_max, 1);
	nh.param<int>("n_c", n_c, 0);
	nh.param<int>("n_q", n_q, 0);
cout << "joints_.size()" << endl << joints_.size() << endl;



  for(int i =0 ; i < joints_.size(); i++)
      joints(i) = joints_[i]*(PI_/180);



	if(n_c <= 0) 
		return 0;

	createStructureKDL();
	setJoints();
	initJacobian();

 Eigen::MatrixXd Skew_Matrix1(3,3);
  Eigen::MatrixXd Skew_Matrix2(3,3);

  Skew_Matrix1(0,0) = Skew_Matrix1(1,1) = Skew_Matrix1(2,2) = 0;
      Skew_Matrix1(0,1) = - contact_points[2]; // -rz    
      Skew_Matrix1(0,2) = contact_points[1];   // ry
      Skew_Matrix1(1,0) = contact_points[2];   // rz
      Skew_Matrix1(2,0) = - contact_points[1]; // -ry
      Skew_Matrix1(1,2) = - contact_points[0]; // -rx
      Skew_Matrix1(2,1) = contact_points[0];   // rx


 Skew_Matrix2(0,0) = Skew_Matrix2(1,1) = Skew_Matrix2(2,2) = 0;
      Skew_Matrix2(0,1) = - contact_points[5]; // -rz    
      Skew_Matrix2(0,2) = contact_points[4];   // ry
      Skew_Matrix2(1,0) = contact_points[5];   // rz
      Skew_Matrix2(2,0) = - contact_points[4]; // -ry
      Skew_Matrix2(1,2) = - contact_points[3]; // -rx
      Skew_Matrix2(2,1) = contact_points[3];   // rx



Eigen::MatrixXd G = MatrixXd::Zero(6, 12);
Eigen::MatrixXd G1 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G2 = MatrixXd::Zero(6,6);

Eigen::MatrixXd R1(3,3);
Eigen::MatrixXd R2(3,3);

R1  << 1, 0, 0,
       0, cos(-90*PPI_/180), -sin(-90*PPI_/180),
       0, sin(-90*PPI_/180), cos(-90*PPI_/180);

R2 << 1, 0, 0,
      0, cos(90*PPI_/180), -sin(90*PPI_/180),
      0, sin(90*PPI_/180), cos(90*PPI_/180);

G1.block<3,3>(0,0) = R1.transpose();
G1.block<3,3>(3,3) = R1.transpose();
G1.block<3,3>(3,0) = Skew_Matrix1 * R1.transpose();

G2.block<3,3>(0,0) = R2.transpose();
G2.block<3,3>(3,3) = R2.transpose();
G2.block<3,3>(3,0) = Skew_Matrix2 * R2.transpose();

G.block<6,6>(0,0) = G1;
G.block<6,6>(0,6) = G2;


Eigen::MatrixXd Jacobian = MatrixXd::Zero(6*n_c, n_q);

  
jnt_to_jac_right->JntToJac(jointpositions_right, jacobian_right);
jnt_to_jac_left->JntToJac(jointpositions_left, jacobian_left);
    
Jacobian.block<6,8>(0,0) = jacobian_right.data;
Jacobian.block<6,8>(6,0) = jacobian_left.data;
  	
  
Eigen::MatrixXd R_c = MatrixXd::Zero(3*n_c,3*n_c);
    int k = 0; 
    for(int i = 0 ; i < n_c ; i++)
    { 
      R_c.block<3,3>(k,k) = MatrixXd::Identity(3,3);
      k += 3;
    }

    Eigen::VectorXd f_c_(contact_forces.size());
    for(int i = 0 ; i < contact_forces.size() ; i++ )
      f_c_(i) = contact_forces[i];

    Eigen::MatrixXd Contact_Stiffness_Matrix = MatrixXd::Zero(3,3);   // Kis
      for(int i = 0 ; i < Contact_Stiffness_Matrix.rows() ; i++) //Kis
          Contact_Stiffness_Matrix(i,i) = contact_stiffness;


    Eigen::MatrixXd Joint_Stiffness_Matrix = MatrixXd::Zero(n_q,n_q);    // Kp            
      for(int j = 0 ; j < Joint_Stiffness_Matrix.rows() ; j++) //Kp
          Joint_Stiffness_Matrix(j,j) = joint_stiffness;
  
  //quality_i = quality_pcr_pgr_5(f_c_, G_c, Jacobian, R_c, Contact_Stiffness_Matrix, Joint_Stiffness_Matrix, mu, f_i_max);
 // quality_i = quality_pcr_pgr_5(f_c_, G_b, Jacobian, R_c, Contact_Stiffness_Matrix, Joint_Stiffness_Matrix, mu, f_i_max);



  quality_i = quality_pcr_pgr_5(f_c_, G, Jacobian, R_c, Contact_Stiffness_Matrix, Joint_Stiffness_Matrix, mu, f_i_max);


  cout << " quality : " << quality_i << endl;

	ros::spinOnce();
	return 0;
}