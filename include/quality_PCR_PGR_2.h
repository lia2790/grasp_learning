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


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


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




inline double quality_measures_PCR_PGR(Eigen::VectorXd &f , Eigen::MatrixXd &G , Eigen::MatrixXd &J , Eigen::MatrixXd &Kis_ , Eigen::MatrixXd &Kp , Eigen::MatrixXd &S ,Eigen::VectorXd &synergy , double &mu_friction , double f_i_max, int setting)
{
	// G ---> grasp matrix 6x6nc
	// J ---> hand_jacobian 6nc,nq



	int n_c = f.size()/6;  // number of contacts
	int n_q = J.cols(); // number of joints
	int n_z = synergy.size();  // number of synergie
	// double synergy = 1;


	double Kx = Kis_(0,0);
	double Ky = Kis_(1,1);
	double Kz = Kis_(2,2);


	// double f_i_max = 20; 
	// double mu_friction = 1;
	double PCR_PGR;


	//Eigen::VectorXd f(6*n_c); // contact force
	Eigen::VectorXd H_f(3*n_c); // contact force with type model of contact
	Eigen::VectorXd f_y(3*n_c); // controllable contact force

	Eigen::VectorXd w(6);

	Eigen::MatrixXd H(3*n_c,6*n_c);
	Eigen::MatrixXd H_(3,6);


	//Eigen::MatrixXd G(6, 6*n_c);
	//Eigen::MatrixXd J(6*n_c, n_q);
	Eigen::MatrixXd G_H_t(6,3*n_c); // Grasp Matrix 
	Eigen::MatrixXd H_J(3*n_c,n_q); // Hand Jacobian 
	//calculation weighted right pseudoinverse
	Eigen::MatrixXd G_inv;
	Eigen::MatrixXd G_;
	// calculation the grasp stiffness matrix K(Cj)
	Eigen::MatrixXd K_(3*n_c,3*n_c);
	

	Eigen::MatrixXd K(3*n_c,3*n_c); // grasp stiffness matrix
	Eigen::MatrixXd Kis(3,3);
	Eigen::MatrixXd Ks(3*n_c,3*n_c); // contact stiffness matrix
	//Eigen::MatrixXd Kp(n_q,n_q); // joint stiffness matrix

	//Eigen::MatrixXd S(n_q,n_z); // Synergie matriz for underactuacted hand
	Eigen::VectorXd f_(3*n_c) ;
	Eigen::MatrixXd G_r_k(3*n_c,6); // weighted right pseudoinverse of G	
	Eigen::VectorXd d_f(3*n_c);




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////          apply the type of contact : HARD FINGER CONTACTS             //////////////////////////////////////////////

 


	// calculation the selection matrix for n_contact points
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


	// apply the type of contact and i obtain the grasp matrix and hand jacobian 
	G_H_t = G * H.transpose();
	H_J = H * J ;
	H_f = H * f ;





	int count = 1;
	for ( int i = 0 ; i < H_f.size() ; i++)
		if( H_f(i) == 0 )
			count ++ ;

	if(count == H_f.size()) // I don't have any contact point 
		return -1;





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





	switch(setting)
	{

		case 0 :  // direct calc PCR very simple
			{
				f_ = H_f ;

				int s = 0;
				for(int i = 0 ; i < n_c ; i++)
				{	Ks.block<3,3>(s,s) = Kis_; s+=6;  }



				K_ = Ks.inverse() + H_J * Kp.inverse() * H_J.transpose() ;
				K  = K_.inverse();
				G_ = G_H_t * K * G_H_t.transpose();
				G_inv = G_.inverse();
				G_r_k = K * G.transpose() * G_inv; 

				break;
			}
		case 1 :  // PGR with Cj fixed we have just a Cj from the dataset
			{
				// calculation Ks(Cj)
				int s_ = 0; // step for component
				for( int i = 0 ; i < n_c ; i++ )
				{
					Kis(0,0) = 0;
					Kis(1,1) = 0;
					Kis(2,2) = 0;

					if( H_f(i+s_+3) >= 0 )
						if(  sqrt( H_f(i+s_+0)*H_f(i+s_+0) + H_f(i+s_+1)*H_f(i+s_+1)) <= mu_friction * H_f(i+s_+2) )
						{
							Kis(0,0) =  Kx;
							Kis(1,1) =  Ky;
							Kis(2,2) =  Kz;
						}
						else
							Ks(2,2) = Kz;

					Ks.block<3,3>(s_,s_) = Kis;
					s_ += 3;
				}	

				
				K_ = Ks.inverse() + H_J * Kp.inverse() * H_J.transpose() ;
				K  = K_.inverse();



				// evaluation the constraint N(K(Cj)*Gt) = 0 
				// it must be satisfied to immobilize the object
				Eigen::MatrixXd Kcj_Gt = K * G_H_t.transpose();
				FullPivLU<MatrixXd> lu(Kcj_Gt);
				Eigen::MatrixXd Null_Kcj_Gt = lu.kernel();


				bool Matrix_is_Zero = true;

				for ( int i = 0 ; i < Null_Kcj_Gt.rows() ; i++ )
					for ( int j = 0 ; j < Null_Kcj_Gt.cols() ; j++)
						if( Null_Kcj_Gt(i,j) != 0 )
						{	Matrix_is_Zero = false; break; }


				if( !Matrix_is_Zero ) // condition-constrain of PGR is not satisfy : N(K(Cj)*Gt) = 0 
					return 0;

				//calculation of a basis for the subspace of the controllable internal force
				// with synergy
				Eigen::MatrixXd I = MatrixXd::Identity(3*n_c, 3*n_c);
				Eigen::MatrixXd F(3*n_c,n_q);
				Eigen::MatrixXd E;




				G_ = G_H_t * K * G_H_t.transpose();
				G_inv = G_.inverse();
				G_r_k = K * G.transpose() * G_inv; 


				F = ( I - G_r_k * G ) * K * H_J ; // and maps independently controlled joint reference displacements δqr’s into active internal forces
												  // if we assume that we have a perfect rigid joint δqr = δq
				E = F * S ;
				w = - G_H_t * H_f ; // calculatin of external wrench
				f_y = - G_r_k * w + E * synergy ; // calculation of the controllable contact forces


				f_ = f_y ;


				break;
			}
			default : return  -2;
	}


	

	// now we are ready to search the minimum value of d(f_y) vector
	Eigen::VectorXd f_i(3);
	Eigen::VectorXd n_i(3);
	Eigen::VectorXd f_i_n(1);
	int step = 0;
	int step_ = 0;
	double f_i_ ;

	Eigen::MatrixXd Rotation(3,3);



	for(int i = 0 ; i < (f_.size()/3)  ; i++)
	{
		f_i(0) = f_(step+0);
		f_i(1) = f_(step+1);
		f_i(2) = f_(step+2);

		Rotation = G.block<3,3>(0, step_);

		// the normal of the surface is alligned with the z-axis {o,t,n}
		n_i(0) = Rotation(0,2);
		n_i(1) = Rotation(1,2);
		n_i(2) = Rotation(2,2);
		
		f_i_ = f_i(0)+f_i(1)+f_i(2);
		f_i_n = f_i * n_i.transpose();


		d_f(step+0)  =  f_i_n(0);	
		d_f(step+1)  =  mu_friction*f_i_n(0) - (sin(acos(f_i_n(0)/f_i_)));		
		d_f(step+2)  =  f_i_max - f_i.norm();

		step  += 3;
		step_ += 6;
	}



	double d_min = d_f(0);	
	for(int i = 0 ; i < d_f.size() ; i++)
		if( d_min > d_f(i) ) 
			d_min = d_f(i);



	Eigen::VectorXd Singular ;
	JacobiSVD<MatrixXd> svd(G_r_k, ComputeThinU | ComputeThinV);  
    Singular = svd.singularValues();

    double sigma_max = Singular[0];

    PCR_PGR = d_min / sigma_max;
	
	return PCR_PGR;
}