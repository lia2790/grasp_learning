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


using namespace std;
using namespace Eigen;
using namespace KDL;




int main (int argc, char **argv)
{

	ros::init(argc, argv, "Grasp_quality");	// ROS node
	ros::NodeHandle nh;


	int n_c = 1; //number of contacts
	int n_q = 19; //number of joints
	int n_z = 1; // number of synergie
	int synergy = 1;


	double Kx = 1;
	double Ky = 1;
	double Kz = 1;


	double f_i_max = 20;
	double mu_friction = 1;
	double PCR;





	Eigen::VectorXd f(6*n_c); // contact force
	Eigen::VectorXd H_f(3*n_c); // contact force with type model of contact
	Eigen::VectorXd f_y(3*n_c); // controllable contact force

	Eigen::VectorXd w(6);

	Eigen::MatrixXd G(6, 6*n_c);
	Eigen::MatrixXd G_H_t(6,3*n_c); // Grasp Matrix
	Eigen::MatrixXd J(6*n_c, n_q); 
	Eigen::MatrixXd H_J(3*n_c,n_q); // Hand Jacobian
	Eigen::MatrixXd G_r_k(3*n_c,6); // weighted right pseudoinverse of G


	Eigen::MatrixXd K(3*n_c,3*n_c); // grasp stiffness matrix
	Eigen::MatrixXd Kis(3,3);
	Eigen::MatrixXd Ks(3*n_c,3*n_c); // contact stiffness matrix
	Eigen::MatrixXd Kp(n_q,n_q); // joint stiffness matrix

	Eigen::MatrixXd S(n_q,n_z); // Synergie matriz for underactuacted hand


	Eigen::VectorXd d_f(3*n_c);


	// calculation Ks(Cj)
	int s = 0; // step for component
	for( int i = 0 ; i < n_c ; i++ )
	{
		Kis(0,0) = 0;
		Kis(1,1) = 0;
		Kis(2,2) = 0;

		if( H_f(i+s+3) >= 0 )
			if(  sqrt( H_f(i+s+0)*H_f(i+s+0) + H_f(i+s+1)*H_f(i+s+1)) <= mu_friction * H_f(i+s+2) )
			{
				Kis(0,0) =  Kx;
				Kis(1,1) =  Ky;
				Kis(2,2) =  Kz;
			}
			else
				Ks(2,2) = Kz;
		
		Ks.block<3,3>(s,s) = Kis;
		s += 3;
	}	

	// calculation the grasp stiffness matrix K(Cj)
	Eigen::MatrixXd K_(3*n_c,3*n_c);
	K_ = Ks.inverse() + H_J * Kp.inverse() * H_J.transpose() ;
	K  = K_.inverse();



	// evaluation the constraint N(K(Cj)*Gt) = 0 
	// it must be satisfied to immobilize the object
	Eigen::MatrixXd Kcj_Gt = K * G_H_t.transpose();
	FullPivLU<MatrixXd> lu(Kcj_Gt);
	Eigen::MatrixXd Null_Kcj_Gt = lu.kernel();


	bool Matrix_is_Zero = true;

	for (int i = 0 ; i < Null_Kcj_Gt.rows() ; i++ )
		for ( int j = 0 ; j < Null_Kcj_Gt.cols() ; j++)
			if( Null_Kcj_Gt(i,j) != 0 )
			{	Matrix_is_Zero = false; break; }


	if( !Matrix_is_Zero ) // condition-constrain of PGR is not satisfy
		return 0;



	Eigen::MatrixXd H_(3,6);

	H_ << 1, 0, 0, 0, 0, 0,
		  0, 1, 0, 0, 0, 0,
		  0, 0, 1, 0, 0, 0;


	Eigen::MatrixXd H(3*n_c,6*n_c);



	int i = 0;
	int j = 0;
	for(int n = 0 ;  n < n_c ; n++)
	{
		H.block<3,6>(i,j) = H_;
		i += 3;
		j += 6;
	}




	//define the sinergie matrix 
	for(int j = 0 ; j < n_q ; j++)
		for(int i = 0; i < n_z ; i++)
			S(j,i) = 1;



	
	// apply the kind of the point id contact model
	G_H_t = G * H.transpose();
	H_J = H * J ;
	H_f = H * f ;


	// calculatin of external wrench
	w = - G_H_t * H_f;



	//calculation weighted right pseudoinverse
	Eigen::MatrixXd G_inv;
	Eigen::MatrixXd G_;
	G_ = G_H_t * K * G_H_t.transpose();
	G_inv = G_.inverse();
	G_r_k = K * G.transpose() * G_inv; 


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////


	//calculation of a basis for the subspace of the controllable internal forces
	// with null of GHt
	FullPivLU<MatrixXd> lu(G_H_t);
	Eigen::MatrixXd E_ = lu.kernel();
	Eigen::VectorXd y_(E_.cols());

	for(int i = 0; i < y_.size(); i++)
		y_(i)=1;


	f_y = - G_r_k * w + E_* y_ ; //calculation of the controllable contact forces


	

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////777777777777777777777777777777777777
	///////////////////////////////////////////////////////////////////////////////////////


	//calculation of a basis for the subspace of the controllable internal force
	// with synergy
	Eigen::MatrixXd I = MatrixXd::Identity(3*n_c, 3*n_c);
	Eigen::MatrixXd F(3*n_c,n_q);
	Eigen::MatrixXd E;
	Eigen::VectorXd y(1);

	y(0) = synergy;



	F = ( I - G_r_k * G ) * K * H_J; // and maps independently controlled joint reference displacements δqr’s into active internal forces

	E = F * S ;
	
	f_y = - G_r_k * w + E * y ; // calculation of the controllable contact forces




	// search the minimum value of d(f_y) vector
	Eigen::VectorXd f_i(3);
	Eigen::VectorXd n_i(3);
	Eigen::VectorXd f_i_n(1);
	int step = 0;
	double f_i_ ;
	

	

	for(int i = 0 ; i < (f_y.size()/3)  ; i++)
	{
		f_i(0) = f_y(step+0);
		f_i(1) = f_y(step+1);
		f_i(2) = f_y(step+2);

		
		f_i_ = f_i(0)+f_i(1)+f_i(2);
		f_i_n = f_i * n_i.transpose();


		d_f(step+0)  =  f_i_n(0);	
		d_f(step+1)  =  mu_friction*f_i_n(0) - (sin(acos(f_i_n(0)/f_i_)));		
		d_f(step+2)  =  f_i_max - f_i.norm();

		step += 3;
	}



	double d_min = d_f(0);
	
	for(int i = 0 ; i < d_f.size() ; i++)
		if( d_min > d_f(i) ) 
			d_min = d_f(i);



	Eigen::VectorXd Singular ;
	JacobiSVD<MatrixXd> svd(G_r_k, ComputeThinU | ComputeThinV);  
    Singular = svd.singularValues();

    double sigma_max = Singular[0];

    PCR = d_min / sigma_max;
			



	cout << " YEAH ENJOY " << endl;
	cout << "   fine   " << endl;


	ros::spin();
	return 0;
}