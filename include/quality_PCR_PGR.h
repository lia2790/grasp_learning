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

using namespace Eigen;


inline double quality_PCR_PGR(Eigen::MatrixXd &Force_Contact, Eigen::MatrixXd &Grasp_Matrix, Eigen::MatrixXd &Hand_Jacobian, Eigen::MatrixXd &Contact_Stiffness_Matrix, Eigen::MatrixXd &Joint_Stiffness_Matrix)
{



	double f_i_max = 1 ; // upper limit
	double mu_friction = 1 ; // friction
	double f_i_ ;
	double f_i_n ;
	Eigen::VectorXd f_i(3);
	int n_c = Force_Contact.rows();

	Eigen::MatrixXd d_f(n_c,3);


	for(int i = 0; i < Force_Contact.rows() ; i++) // for each contact point
	{
		f_i(0) = Force_Contact(i,0);
		f_i(1) = Force_Contact(i,1);
		f_i(2) = Force_Contact(i,2);

		f_i_n = f_i(2);
		f_i_ = f_i(0)+f_i(1)+f_i(2);

		d_f(i,0)  =  f_i_n;	// d_1_c = f_1_n = f_1_z
		d_f(i,1)  =  mu_friction*f_i_n - (sin(acos(f_i_n/f_i_)));		
		d_f(i,2)  =  f_i_max - f_i.norm();
	}


	double d_min = Force_Contact(0,0);


	for(int i = 0 ; i < Force_Contact.rows() ; i++)
		for(int j = 0 ; j < Force_Contact.cols() ; j++)
			if( d_min > Force_Contact(i,j)) 
				d_min = Force_Contact(i,j);

	Eigen::MatrixXd G_r_k;
	Eigen::MatrixXd K;

	K = Contact_Stiffness_Matrix.inverse() + Hand_Jacobian * Joint_Stiffness_Matrix.inverse() * Hand_Jacobian.transpose() ;
	G_r_k = K * Grasp_Matrix.transpose() * Grasp_Matrix * K * Grasp_Matrix.transpose();


	Eigen::VectorXd Singular ;
	JacobiSVD<MatrixXd> svd(G_r_k, ComputeThinU | ComputeThinV);  
    Singular = svd.singularValues();

    double sigma_max = Singular[0];

    return d_min / sigma_max ;
}