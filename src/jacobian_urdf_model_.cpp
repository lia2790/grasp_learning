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


#include <cmath>
#include <ctime>
#include <time.h>




using namespace std;
using namespace Eigen;
using namespace KDL;



int number_of_chain = 5; // 5 finger



int main (int argc, char **argv)
{

	ros::init(argc, argv, "Soft_Hand_Jacobian");	// ROS node
	ros::NodeHandle nh;



  std::string root_name = "right_hand_palm_link" ;
  std::string end_chain_name[number_of_chain] ;


  end_chain_name[0] = "right_hand_thumb_distal_link";
  end_chain_name[1] = "right_hand_index_distal_link";
  end_chain_name[2] = "right_hand_middle_distal_link";
  end_chain_name[3] = "right_hand_ring_distal_link";
  end_chain_name[4] = "right_hand_little_distal_link";



  double input_joint_thumb[] = { 0.2, 0.2, 0, 0, 0 };
  double input_joint_index[] = { 0.2, 0.2, 0, 0, 0, 0, 0 };
  double input_joint_middle[] = { 0.2, 0.2, 0, 0, 0, 0, 0 };
  double input_joint_ring[] = { 0.2, 0.2, 0, 0, 0, 0, 0 };
  double input_joint_little[] = { 0.2, 0.2, 0, 0, 0, 0, 0 };








  KDL::Tree hand_tree;

  KDL::Chain chain0_thumb;
  KDL::Chain chain1_index;
  KDL::Chain chain2_middle;
  KDL::Chain chain3_ring;
  KDL::Chain chain4_little;


  KDL::Jacobian jacobian0_thumb;
  KDL::Jacobian jacobian1_index;
  KDL::Jacobian jacobian2_middle;
  KDL::Jacobian jacobian3_ring;
  KDL::Jacobian jacobian4_little;


  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_0;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_1;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_2;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_3;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_4;


  


  std::string robot_desc_string;
  nh.param("robot_description", robot_desc_string, string());  // robot description is the name in the launch file 
  if (!kdl_parser::treeFromString(robot_desc_string, hand_tree))
  {     ROS_ERROR("Failed to construct kdl tree");
        return false;       }



  hand_tree.getChain(root_name, end_chain_name[0], chain0_thumb);      // da root a leaf
  hand_tree.getChain(root_name, end_chain_name[1], chain1_index); 
  hand_tree.getChain(root_name, end_chain_name[2], chain2_middle);
  hand_tree.getChain(root_name, end_chain_name[3], chain3_ring);
  hand_tree.getChain(root_name, end_chain_name[4], chain4_little);


  int dim_hand_tree = hand_tree.getNrOfJoints();  // 34

  cout << "number_of_joint : " << dim_hand_tree << endl;

  unsigned int nj_0 = chain0_thumb.getNrOfJoints();
  unsigned int nj_1 = chain1_index.getNrOfJoints();
  unsigned int nj_2 = chain2_middle.getNrOfJoints();
  unsigned int nj_3 = chain3_ring.getNrOfJoints();
  unsigned int nj_4 = chain4_little.getNrOfJoints();



  cout << "chain0_thumb - nj_0 = "   << nj_0 << endl;
  cout << "chain1_index - nj_1 = "   << nj_1 << endl;
  cout << "chain2_middle - nj_2 = "  << nj_2 << endl;
  cout << "chain3_ring - nj_3 = "    << nj_3 << endl;
  cout << "chain4_little - nj_4 = "  << nj_4 << endl;
  



  KDL::JntArray jointpositions_0 = JntArray(nj_0);    // thumb
  KDL::JntArray jointpositions_1 = JntArray(nj_1);    // forefinger
  KDL::JntArray jointpositions_2 = JntArray(nj_2);    // middlefinger
  KDL::JntArray jointpositions_3 = JntArray(nj_3);    // ringfinger
  KDL::JntArray jointpositions_4 = JntArray(nj_4);    // littlefinger



  for ( int i = 0 ; i < nj_0 ; i++ )
      jointpositions_0(i) = input_joint_thumb[i];


  for ( int i = 0 ; i < nj_1 ; i++ )
      jointpositions_1(i) = input_joint_index[i];

  
  for ( int i = 0 ; i < nj_2 ; i++ )
      jointpositions_2(i) = input_joint_middle[i];


  for ( int i = 0 ; i < nj_3 ; i++ )
      jointpositions_3(i) = input_joint_ring[i];


  for ( int i = 0 ; i < nj_4 ; i++ )
      jointpositions_4(i) = input_joint_little[i];




  // constructs the kdl solvers in non-realtime
  jnt_to_jac_solver_0.reset(new KDL::ChainJntToJacSolver(chain0_thumb));
  jnt_to_jac_solver_1.reset(new KDL::ChainJntToJacSolver(chain1_index));
  jnt_to_jac_solver_2.reset(new KDL::ChainJntToJacSolver(chain2_middle));
  jnt_to_jac_solver_3.reset(new KDL::ChainJntToJacSolver(chain3_ring));
  jnt_to_jac_solver_4.reset(new KDL::ChainJntToJacSolver(chain4_little));





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



  for(int n = 0 ; n < number_of_chain ; n++)
  {
    cout << " Jacobian_" << n << endl;
    cout << " --------------------------- " << endl;
    cout <<  Jacobian_finger[n] << endl;
    cout << " --------------------------- " << endl;
  }


	cout << " STAMPATO " << endl;
  cout << "   fine   " << endl;



	ros::spin();
	return 0;

}



