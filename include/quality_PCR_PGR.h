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


inline double quality_PCR_PGR(Eigen::MatrixXd &Contact_Force, Eigen::MatrixXd &Grasp_Matrix, Eigen::MatrixXd &Hand_Jacobian, Eigen::MatrixXd &Contact_Stiffness_Matrix, Eigen::MatrixXd &Joint_Stiffness_Matrix, int type_of_contact = 0)
{
	double synergie = 1;
	double number_of_synergies = 1;
	// type_of_contact 0 : hard finger , 1 : soft finger

	std::cout << " sono qui " << std::endl;


	Eigen::MatrixXd H_Selection_Matrix(4,6);
	

	if( !type_of_contact )
	{	
		H_Selection_Matrix.resize(3,6);
		H_Selection_Matrix << 1,0,0,0,0,0,
							  0,1,0,0,0,0,
							  0,0,1,0,0,0;
	}
	else
		H_Selection_Matrix << 1,0,0,0,0,0,
							  0,1,0,0,0,0,
							  0,0,1,0,0,0,
							  0,0,0,0,0,1;
	

	std::cout << " sono qui 2" << std::endl;


	int n_c = Contact_Force.rows();
	int r = H_Selection_Matrix.rows(); // 3
	int c = H_Selection_Matrix.cols(); // 6

	Eigen::MatrixXd H_Selection_Matrix_Contact = MatrixXd::Zero(r*n_c, c*n_c); 


	int h = 0;
	int k = 0;


	std::cout << " sono qui 3 " << std::endl;


	for(int i = 0 ; i < n_c; i++ )
	{	
		H_Selection_Matrix_Contact.block(h,k,r,c) = H_Selection_Matrix ;
		h += H_Selection_Matrix.rows();
		k += H_Selection_Matrix.cols();
	}

	// filter matrices based on the type of contact

	std::cout << " sono qui 4 " << std::endl;


	Eigen::MatrixXd G; // 6x3n_c
	Eigen::MatrixXd J; // 3n_cxn_q
	Eigen::MatrixXd K_s; //3n_cX3n_c
	Eigen::MatrixXd K_p = Joint_Stiffness_Matrix;

	std::cout << Grasp_Matrix.rows() << " --- " << Grasp_Matrix.cols() << " --- " << std::endl;
	std::cout << H_Selection_Matrix_Contact.rows() << " ... " << H_Selection_Matrix_Contact.cols() << std::endl;




	G = Grasp_Matrix * H_Selection_Matrix_Contact.transpose();




	J = H_Selection_Matrix_Contact * Hand_Jacobian;
	K_s = H_Selection_Matrix_Contact * Contact_Stiffness_Matrix * H_Selection_Matrix_Contact.transpose();


	

	int n_q = Hand_Jacobian.size();
	int n_z = number_of_synergies;




	//synergie
	Eigen::MatrixXd S(n_q,n_z);
	Eigen::MatrixXd J_synergie;



	for(int j = 0 ; j < n_q ; j++)
		for(int i = 0; i < n_z ; i++)
			S(j,i) = 1;

		
	J_synergie = J*S;

	
	Eigen::MatrixXd G_r_k;
	Eigen::MatrixXd K;
	Eigen::MatrixXd K_inverse;

	K = K_s.inverse() + J_synergie * K_p.inverse() * J_synergie.transpose() ;
	K_inverse = K.inverse();
	G_r_k = K * G.transpose() * (G * K * G.transpose());



	FullPivLU<MatrixXd> lu(G);
	Eigen::MatrixXd E = lu.kernel();
	Eigen::VectorXd y(E.cols());

	for(int i = 0; i < y.size(); i++)
		y(i)=synergie;



	Eigen::VectorXd w_i(6*n_c);
	Eigen::VectorXd w_i_(3*n_c);
	Eigen::VectorXd w(6);
	Eigen::VectorXd f_y(3*n_c);

	int gap_coord = 0;

	for(int i = 0 ; i < Contact_Force.rows() ; i++)
	{	
		for(int j = 0 ; j < Contact_Force.cols() ; j++)
			w_i(gap_coord + j) = Contact_Force(i,j);
		gap_coord += 6;
	}


	w_i_ = H_Selection_Matrix_Contact * w_i.transpose(); 

	w = - G * w_i_;

	f_y = - G_r_k * w + E * y;




	double f_i_max = 1 ; // upper limit
	double mu_friction = 1 ; // friction
	double f_i_ ;
	double f_i_n ;
	double d_min ;
	Eigen::VectorXd d_f(3*n_c);
	Eigen::VectorXd f_i(3);
	

	int step = 0;

	for(int i = 0; i < (f_y.size()/3) ; i++) // for each contact point
	{
		f_i(0) = f_y(step+0);
		f_i(1) = f_y(step+1);
		f_i(2) = f_y(step+2);

		f_i_n = f_i(2);
		f_i_ = f_i(0)+f_i(1)+f_i(2);

		d_f(step+0)  =  f_i_n;	// d_1_c = f_1_n = f_1_z
		d_f(step+1)  =  mu_friction*f_i_n - (sin(acos(f_i_n/f_i_)));		
		d_f(step+2)  =  f_i_max - f_i.norm();

		step += 3;
	}


	d_min = d_f(0);
	
	for(int i = 0 ; i < d_f.size() ; i++)
		if( d_min > d_f(i)) 
			d_min = d_f(i);
			

	

	Eigen::VectorXd Singular ;
	JacobiSVD<MatrixXd> svd(G_r_k, ComputeThinU | ComputeThinV);  
    Singular = svd.singularValues();

    double sigma_max = Singular[0];

    double PCR = d_min / sigma_max;


    double PGR = PCR;


    std::cout << "REALLYYYY : " << PGR << std::endl;

    return  PGR;

	

}