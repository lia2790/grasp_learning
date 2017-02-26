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

//////////////////////////////////  BOX

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
double quality_i = 0;
double contact_stiffness = 100;
double joint_stiffness = 1; 
double mu = 1.5;
double f_i_max = 200;


//////////////////////////////////////////// ROBOT SCTRUCTURE

KDL::Tree hand_tree("RootName");


  KDL::Chain chain_palm;

  KDL::Chain chain_finger_right_0;
  KDL::Chain chain_finger_left_0;
  KDL::Chain chain_finger_right_1;
  KDL::Chain chain_finger_left_1;
  KDL::Chain chain_finger_right_2;
  KDL::Chain chain_finger_left_2;

  KDL::Chain chain_palm_finger_left_0; // return the result
  KDL::Chain chain_palm_finger_left_1;
  KDL::Chain chain_palm_finger_left_2; 
  KDL::Chain chain_palm_finger_right_0;
  KDL::Chain chain_palm_finger_right_1;
  KDL::Chain chain_palm_finger_right_2;

  KDL::JntArray jointpositions_right_0 = JntArray(8);  
  KDL::JntArray jointpositions_right_1 = JntArray(8);  
  KDL::JntArray jointpositions_right_2 = JntArray(8);  
  KDL::JntArray jointpositions_left_0 = JntArray(8);
  KDL::JntArray jointpositions_left_1 = JntArray(8);
  KDL::JntArray jointpositions_left_2 = JntArray(8);

  KDL::Jacobian jacobian_right_0;
  KDL::Jacobian jacobian_left_0;
  KDL::Jacobian jacobian_right_1;
  KDL::Jacobian jacobian_left_1;
  KDL::Jacobian jacobian_right_2;
  KDL::Jacobian jacobian_left_2;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_right_0;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_left_0;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_right_1;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_left_1; 
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_right_2;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_left_2;

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

  chain_finger_right_0.addSegment(Segment("finger_right_proximal_link_0",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
  chain_finger_right_0.addSegment(Segment("finger_right_distal_link_0",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

  chain_finger_left_0.addSegment(Segment("finger_left_proximal_link_0",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.3))));
  chain_finger_left_0.addSegment(Segment("finger_left_distal_link_0",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

  chain_finger_right_1.addSegment(Segment("finger_right_proximal_link_1",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
  chain_finger_right_1.addSegment(Segment("finger_right_distal_link_1",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

  chain_finger_left_1.addSegment(Segment("finger_left_proximal_link_1",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.3))));
  chain_finger_left_1.addSegment(Segment("finger_left_distal_link_1",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

  chain_finger_right_2.addSegment(Segment("finger_right_proximal_link_2",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
  chain_finger_right_2.addSegment(Segment("finger_right_distal_link_2",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));

  chain_finger_left_2.addSegment(Segment("finger_left_proximal_link_2",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.3))));
  chain_finger_left_2.addSegment(Segment("finger_left_distal_link_2",Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.03))));
    
    

  hand_tree.addChain(chain_palm,"RootName");    
  hand_tree.addChain(chain_finger_right_0,"palm_rot_z_link");
  hand_tree.addChain(chain_finger_left_0, "palm_rot_z_link"); 
  hand_tree.addChain(chain_finger_right_1,"palm_rot_z_link");
  hand_tree.addChain(chain_finger_left_1, "palm_rot_z_link"); 
  hand_tree.addChain(chain_finger_right_2,"palm_rot_z_link");
  hand_tree.addChain(chain_finger_left_2, "palm_rot_z_link"); 



  hand_tree.getChain("RootName", "finger_right_distal_link_0", chain_palm_finger_right_0);
  hand_tree.getChain("RootName", "finger_left_distal_link_0", chain_palm_finger_left_0);
  hand_tree.getChain("RootName", "finger_right_distal_link_1", chain_palm_finger_right_1);
  hand_tree.getChain("RootName", "finger_left_distal_link_1", chain_palm_finger_left_1); 
  hand_tree.getChain("RootName", "finger_right_distal_link_2", chain_palm_finger_right_2);
  hand_tree.getChain("RootName", "finger_left_distal_link_2", chain_palm_finger_left_2);

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
void setJoints(std::vector<double>  joints_in)
{
  for(int i=0 ; i < 6 ; i++)
      jointpositions_right_0(i) = jointpositions_left_0(i) = jointpositions_right_1(i) = jointpositions_left_1(i) = jointpositions_right_2(i) = jointpositions_left_2(i) = joints[i];
 
    jointpositions_right_0(6) = joints_in[6];
    jointpositions_right_0(7) = joints_in[7];

    jointpositions_left_0(6) = joints_in[8];
    jointpositions_left_0(7) = joints_in[9];

    jointpositions_right_1(6) = joints_in[10];
    jointpositions_right_1(7) = joints_in[11];

    jointpositions_left_1(6) = joints_in[12];
    jointpositions_left_1(7) = joints_in[13];

    jointpositions_right_2(6) = joints_in[14];
    jointpositions_right_2(7) = joints_in[15];

    jointpositions_left_2(6) = joints_in[16];
    jointpositions_left_2(7) = joints_in[17];

  jointpositions_right_0.resize(chain_palm_finger_right_0.getNrOfJoints());
  jointpositions_left_0.resize(chain_palm_finger_left_0.getNrOfJoints());

  jointpositions_right_1.resize(chain_palm_finger_right_1.getNrOfJoints());
  jointpositions_left_1.resize(chain_palm_finger_left_1.getNrOfJoints());

  jointpositions_right_2.resize(chain_palm_finger_right_2.getNrOfJoints());
  jointpositions_left_2.resize(chain_palm_finger_left_2.getNrOfJoints());
}



void initJacobian()
{
    jacobian_right_0.resize(chain_palm_finger_right_0.getNrOfJoints());
    jacobian_left_0.resize(chain_palm_finger_left_0.getNrOfJoints());

    jnt_to_jac_right_0.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_right_0));
    jnt_to_jac_left_0.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_left_0));

    jacobian_right_1.resize(chain_palm_finger_right_1.getNrOfJoints());
    jacobian_left_1.resize(chain_palm_finger_left_1.getNrOfJoints());

    jnt_to_jac_right_1.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_right_1));
    jnt_to_jac_left_1.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_left_1));

    jacobian_right_2.resize(chain_palm_finger_right_2.getNrOfJoints());
    jacobian_left_2.resize(chain_palm_finger_left_2.getNrOfJoints());

    jnt_to_jac_right_2.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_right_2));
    jnt_to_jac_left_2.reset(new KDL::ChainJntToJacSolver(chain_palm_finger_left_2));
}



int main (int argc, char **argv)
{

  ros::init(argc, argv, "Test_file_4_finger"); 
  ros::NodeHandle nh;

  


  nh.param<std::vector<double>>("box", box, std::vector<double>{1, 1, 1});
  nh.param<std::vector<double>>("contact_points", contact_points, std::vector<double>{1,1,1,1,1,1});
  nh.param<std::vector<double>>("contact_forces", contact_forces, std::vector<double>{1,1,1,1,1,1});
  nh.param<std::vector<double>>("joints", joints, std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  nh.param<double>("mu", mu, 0.5);
  nh.param<double>("f_i_max", f_i_max, 1);
  nh.param<int>("n_c", n_c, 0);
  nh.param<int>("n_q", n_q, 0);


  if(n_c <= 0) 
    return 0;

  createStructureKDL();
  setJoints(joints);
  initJacobian();

  Eigen::MatrixXd Skew_Matrix1(3,3);
  Eigen::MatrixXd Skew_Matrix2(3,3);
  Eigen::MatrixXd Skew_Matrix3(3,3);
  Eigen::MatrixXd Skew_Matrix4(3,3);
  Eigen::MatrixXd Skew_Matrix5(3,3);
  Eigen::MatrixXd Skew_Matrix6(3,3);

  Skew_Matrix1(0,0) = Skew_Matrix1(1,1) = Skew_Matrix1(2,2) = 0;
      Skew_Matrix1(0,1) = - contact_points[2]; // -rz    
      Skew_Matrix1(0,2) = contact_points[1];   // ry
      Skew_Matrix1(1,0) = contact_points[2];   // rz
      Skew_Matrix1(2,0) = - contact_points[1]; // -ry
      Skew_Matrix1(1,2) = - contact_points[0]; // -rx
      Skew_Matrix1(2,1) = contact_points[0];   // rx
  // left


 Skew_Matrix2(0,0) = Skew_Matrix2(1,1) = Skew_Matrix2(2,2) = 0;
      Skew_Matrix2(0,1) = - contact_points[5]; // -rz    
      Skew_Matrix2(0,2) = contact_points[4];   // ry
      Skew_Matrix2(1,0) = contact_points[5];   // rz
      Skew_Matrix2(2,0) = - contact_points[4]; // -ry
      Skew_Matrix2(1,2) = - contact_points[3]; // -rx
      Skew_Matrix2(2,1) = contact_points[3];   // rx
  // right


  Skew_Matrix3(0,0) = Skew_Matrix3(1,1) = Skew_Matrix3(2,2) = 0;
      Skew_Matrix3(0,1) = - contact_points[8]; // -rz    
      Skew_Matrix3(0,2) = contact_points[7];   // ry
      Skew_Matrix3(1,0) = contact_points[8];   // rz
      Skew_Matrix3(2,0) = - contact_points[7]; // -ry
      Skew_Matrix3(1,2) = - contact_points[6]; // -rx
      Skew_Matrix3(2,1) = contact_points[6];   // rx
  // up


 Skew_Matrix4(0,0) = Skew_Matrix4(1,1) = Skew_Matrix4(2,2) = 0;
      Skew_Matrix4(0,1) = - contact_points[11]; // -rz    
      Skew_Matrix4(0,2) = contact_points[10];   // ry
      Skew_Matrix4(1,0) = contact_points[11];   // rz
      Skew_Matrix4(2,0) = - contact_points[10]; // -ry
      Skew_Matrix4(1,2) = - contact_points[9]; // -rx
      Skew_Matrix4(2,1) = contact_points[9];   // rx
  // down

  Skew_Matrix5(0,0) = Skew_Matrix5(1,1) = Skew_Matrix5(2,2) = 0;
      Skew_Matrix5(0,1) = - contact_points[14]; // -rz    
      Skew_Matrix5(0,2) = contact_points[13];   // ry
      Skew_Matrix5(1,0) = contact_points[14];   // rz
      Skew_Matrix5(2,0) = - contact_points[13]; // -ry
      Skew_Matrix5(1,2) = - contact_points[12]; // -rx
      Skew_Matrix5(2,1) = contact_points[12];   // rx
  // front of


 Skew_Matrix6(0,0) = Skew_Matrix6(1,1) = Skew_Matrix6(2,2) = 0;
      Skew_Matrix6(0,1) = - contact_points[17]; // -rz    
      Skew_Matrix6(0,2) = contact_points[16];   // ry
      Skew_Matrix6(1,0) = contact_points[17];   // rz
      Skew_Matrix6(2,0) = - contact_points[16]; // -ry
      Skew_Matrix6(1,2) = - contact_points[15]; // -rx
      Skew_Matrix6(2,1) = contact_points[15];   // rx
  // behind



Eigen::MatrixXd G_c_t = MatrixXd::Zero(6, 6*n_c);
Eigen::MatrixXd G1 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G2 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G3 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G4 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G5 = MatrixXd::Zero(6,6);
Eigen::MatrixXd G6 = MatrixXd::Zero(6,6);

Eigen::MatrixXd R1(3,3);
Eigen::MatrixXd R2(3,3);
Eigen::MatrixXd R3(3,3);
Eigen::MatrixXd R4(3,3);
Eigen::MatrixXd R5(3,3);
Eigen::MatrixXd R6(3,3);

R1 << 1, 0, 0,
       0, cos(-90*PPI_/180), -sin(-90*PPI_/180),
       0, sin(-90*PPI_/180), cos(-90*PPI_/180);
// left

R2 << 1, 0, 0,
      0, cos(90*PPI_/180), -sin(90*PPI_/180),
      0, sin(90*PPI_/180), cos(90*PPI_/180);
// right

R3 << 1, 0, 0,
      0, cos(180*PPI_/180), -sin(180*PPI_/180),
      0, sin(180*PPI_/180), cos(180*PPI_/180);
// up

R4 << 1, 0, 0,
      0, 1, 0,
      0, 0, 1;
// down

R5 <<  cos(-90*PPI/180),  0, sin(-90*PPI/180),
                   0   ,  1,    0 ,
      -sin(-90*PPI/180),  0, cos(-90*PPI/180);
//front of

R6 << cos(90*PPI/180),  0, sin(90*PPI/180),
                 0   ,  1,    0 ,
      -sin(90*PPI/180), 0, cos(90*PPI/180);
//behind




G1.block<3,3>(0,0) = R1.transpose();
G1.block<3,3>(3,3) = R1.transpose();
G1.block<3,3>(3,0) = Skew_Matrix1 * R1.transpose();

G2.block<3,3>(0,0) = R2.transpose();
G2.block<3,3>(3,3) = R2.transpose();
G2.block<3,3>(3,0) = Skew_Matrix2 * R2.transpose();

G3.block<3,3>(0,0) = R3.transpose();
G3.block<3,3>(3,3) = R3.transpose();
G3.block<3,3>(3,0) = Skew_Matrix3 * R3.transpose();

G4.block<3,3>(0,0) = R4.transpose();
G4.block<3,3>(3,3) = R4.transpose();
G4.block<3,3>(3,0) = Skew_Matrix4 * R4.transpose();

G5.block<3,3>(0,0) = R5.transpose();
G5.block<3,3>(3,3) = R5.transpose();
G5.block<3,3>(3,0) = Skew_Matrix5 * R5.transpose();

G6.block<3,3>(0,0) = R6.transpose();
G6.block<3,3>(3,3) = R6.transpose();
G6.block<3,3>(3,0) = Skew_Matrix6 * R6.transpose();

G_c_t.block<6,6>(0,0) = G1;
G_c_t.block<6,6>(0,6) = G2;
G_c_t.block<6,6>(0,12) = G3;
G_c_t.block<6,6>(0,18) = G4;
G_c_t.block<6,6>(0,24) = G5;
G_c_t.block<6,6>(0,30) = G6;



Eigen::MatrixXd Jacobian = MatrixXd::Zero(6*n_c,n_q);

jnt_to_jac_right_0->JntToJac(jointpositions_right_0, jacobian_right_0);
jnt_to_jac_left_0->JntToJac(jointpositions_left_0, jacobian_left_0);

jnt_to_jac_right_1->JntToJac(jointpositions_right_1, jacobian_right_1);
jnt_to_jac_left_1->JntToJac(jointpositions_left_1, jacobian_left_1);

jnt_to_jac_right_2->JntToJac(jointpositions_right_2, jacobian_right_2);
jnt_to_jac_left_2->JntToJac(jointpositions_left_2, jacobian_left_2);


    
Jacobian.block<6,8>(0,0) = jacobian_right_0.data;                  // first contact - first finger

Jacobian.block<6,6>(6,0) = jacobian_left_0.data.block<6,6>(0,0);   // second contact - second finger
Jacobian.block<6,2>(6,8) = jacobian_left_0.data.block<6,2>(0,6);

Jacobian.block<6,6>(12,0) = jacobian_right_1.data.block<6,6>(0,0); // third contact - third finger
Jacobian.block<6,2>(12,10) = jacobian_right_1.data.block<6,2>(0,6);

Jacobian.block<6,6>(18,0) = jacobian_left_1.data.block<6,6>(0,0);  // four-th contact - four-th finger
Jacobian.block<6,2>(18,12) = jacobian_left_1.data.block<6,2>(0,6);
 
Jacobian.block<6,6>(24,0) = jacobian_right_2.data.block<6,6>(0,0);  // five-th contact - five-th finger
Jacobian.block<6,2>(24,14) = jacobian_right_2.data.block<6,2>(0,6);
 
Jacobian.block<6,6>(30,0) = jacobian_left_2.data.block<6,6>(0,0);  // six-th contact - six-th finger
Jacobian.block<6,2>(30,16) = jacobian_left_2.data.block<6,2>(0,6);
 




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



  quality_i = quality_pcr_pgr_5(f_c_, G_c_t, Jacobian, R_c, Contact_Stiffness_Matrix, Joint_Stiffness_Matrix, mu, f_i_max);


  cout << " quality : " << quality_i << endl;

  ros::spinOnce();
  return 0;
}