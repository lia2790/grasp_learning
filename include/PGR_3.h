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


#include <math.h>
#include <stdio.h>
#include <ctime>
#include <time.h>


using namespace std;
using namespace Eigen;
using namespace KDL;



inline double quality_pcr_pgr_3(Eigen::VectorXd &f , Eigen::MatrixXd &G_ , Eigen::MatrixXd &J_ , Eigen::MatrixXd &R_ , Eigen::MatrixXd &Kis , Eigen::MatrixXd &Kp , double mu , double f_i_max)
{
	// assumed that the argument it is already modelled with contact model : HARD CONTACT FINGER
	// the vector of force are espressed in contact point --- frame in contact point {C}  --- > f_c
	// the grasp matrix so are b_G_c 

	// cout << " R_ " << endl << R_ << endl; 
	// cout << " f " << endl << f << endl;

	int n_c = f.size()/6;
	double Kx = Kis(0,0);
	double Ky = Kis(1,1);
	double Kz = Kis(2,2);


	int n_g = 0;
	int s_ = 0;
	for( int i = 0 ; i < n_c ; i++ )
	{		
		if( f(s_+2) >= 0 )   
		{	if(  ((f(s_+0)*f(s_+0) + f(s_+1)*f(s_+1)) / mu ) <=  ( f(s_+2)* f(s_+2)) )
			{	
				n_g += 3;
			}
			else
			{	
				n_g +=1;
			}
		}
	}	
	


	Eigen::MatrixXd Ks = MatrixXd::Zero(n_g, n_g);
	Eigen::MatrixXd H = MatrixXd::Zero(n_g, 6*n_c);

	Eigen::MatrixXd H_3(3, 6);
	Eigen::MatrixXd H_1(1, 6);


	H_3 << 1,0,0,0,0,0,
		   0,1,0,0,0,0,
		   0,0,1,0,0,0;

	H_1 << 0,0,1,0,0,0;


	int now = 0;
	int now_ = 0;
	int r_now = 0;
	int c_now = 0;

	Eigen::MatrixXd K = MatrixXd::Zero(3*n_c,3*n_c);
	Eigen::MatrixXd K_ = MatrixXd::Zero(3*n_c,3*n_c);
	Eigen::MatrixXd Kis_ = MatrixXd::Zero(3,3);

///////////////////////////////////////////////// build H matrix and Ks depending in which state the contact force are
	


	for( int i = 0 ; i < n_c ; i++ )
	{
		Kis_(0,0) = 0;
		Kis_(1,1) = 0;
		Kis_(2,2) = 0;
			
		if( f(s_+2) >= 0 )   
		{
			if(  ((f(s_+0)*f(s_+0) + f(s_+1)*f(s_+1)) / mu ) <=  ( f(s_+2)* f(s_+2)) )
			{	
				Kis_(0,0) =  Kx;
				Kis_(1,1) =  Ky;
				Kis_(2,2) =  Kz; 

				
				Eigen::MatrixXd R_app = R_.block<3,3>(now_,now_);

				Eigen::MatrixXd K_app = R_app.transpose() * Kis_ * R_app;

				Ks.block<3,3>(now,now) = K_app; 
				H.block<3,6>(r_now, c_now) = H_3;

				
				now += 3;
				r_now += 3;
				c_now += 6;
			}
			else
			{	
				Ks(now,now) = Kz; 
				H.block<1,6>(r_now, c_now) = H_1;

				now += 1;
				r_now += 1;
				c_now += 6;
			}
		}
		else
			return -90;
		

		now_ += 3;
	}	


	cout << " Ks : " << endl << Ks << endl;
	cout << " H : " << endl << H << endl;



//////////////////////////////////////////////////////////////// Filter the G and J matrix


	Eigen::MatrixXd G = G_ * H.transpose();
	Eigen::MatrixXd J = H * J_;


	// Ks is already inverse matrix because i assumed that it is a diagonal matrix

	
	Eigen::MatrixXd K_inv = Ks.inverse();

	K_ = K_inv + J * Kp.inverse() * J.transpose();
	K = K_.inverse(); 

	
	Eigen::MatrixXd Kcj_Gt = K*G.transpose();	
	FullPivLU<MatrixXd> lu(Kcj_Gt);		
	Eigen::MatrixXd Null_Kcj_Gt = lu.kernel(); 



	bool Matrix_is_Zero = true;				
	for ( int i = 0 ; i < Null_Kcj_Gt.rows() ; i++ )
		for ( int j = 0 ; j < Null_Kcj_Gt.cols() ; j++)
			if( Null_Kcj_Gt(i,j) != 0 )
			{	Matrix_is_Zero = false; break; } 
				
	if( !Matrix_is_Zero ) // condition-constrain of PGR is not satisfy : N(K(Cj)*Gt) != 0 
		return -27;





	Eigen::VectorXd d_f = VectorXd::Zero(3*n_c);
	Eigen::VectorXd f_i(3);

	int step = 0;
	double f_i_ = 0;

	
	for(int i = 0; i < n_c ; i++)  // normal component is a z-axis of each contact force
	{
		f_i(0) = f(step+0);
		f_i(1) = f(step+1);
		f_i(2) = f(step+2);

		f_i_ = f_i(0)+f_i(1)+f_i(2);

		d_f(step+0)  =  f_i(2);	
		d_f(step+1)  =  mu*f_i(2) - f_i_*(sin(acos(f_i(2)/f_i_)));	// da nan
		d_f(step+2)  =  f_i_max - f_i.norm();

		step += 3;
	}

	double d_min = d_f(0);	
	for(int i = 0 ; i < d_f.size() ; i++)
		if( d_min > d_f(i) ) 
			d_min = d_f(i);

	

	Eigen::MatrixXd GKGt = G*K*G.transpose(); 
	Eigen::MatrixXd GKGt_inv = GKGt.inverse();
	Eigen::MatrixXd G_r_k = K * G.transpose() * GKGt_inv; 



	Eigen::VectorXd Singular ;
	JacobiSVD<MatrixXd> svd(G_r_k, ComputeThinU | ComputeThinV);  
    Singular = svd.singularValues();

    double sigma_max = Singular[0];
	double PCR_PGR = d_min / sigma_max;





	// cout << "Kcj_Gt : " << endl;
	// cout << Kcj_Gt << endl;
	// cout << " Null_Kcj_Gt : " << endl; 
	// cout << Null_Kcj_Gt << endl;
	// cout << " df : " << endl << d_f << endl;
    // cout << "f :" << endl;
    // cout << f << endl;
    // cout << "G_r_k_ : " << endl;
    // cout << G_r_k << endl;
    cout << " d(f) : " << endl;
    cout << d_f << endl;
    cout << "-------------------------------" << endl;
    cout << " d_min : " << d_min << endl;
    cout << " singularValues : " << endl;
    cout << Singular << endl;
    cout << " sigma_max : " << sigma_max << endl;
    cout << " PCR_PGR : " << PCR_PGR << endl;
    cout << "-------------------------------" << endl;
	
	return PCR_PGR;
}