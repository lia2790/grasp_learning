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

const double PPI  = 3.141592653589793238463;



inline void normal_component(Eigen::MatrixXd &R_in, double x_r, double y_r, double z_r, double x_i, double y_i, double z_i)
{
	// In R viene restituita la rotazione che porta
	// dal sistema di riferimento posto sulla faccia della box
	// 			con l'asse z positivo orientato verso l'interno dell'oggetto, 
	// al sistema di riferimento posto al centro della box con l'asse zeta 
	// 			rivolto verso l'alto e l'asse y con direzione positiva verso destra



	//  x_r y_r z_r sono le dimensioni della box dimezzate , 
	// esempio:
	//   se una scatola ha dimensione : { x, y, z } = { 4 , 6, 8 } 
	//   allora questa funzione deve 
	//     essere chiamata con          { x_r  = x / 2 = 2
	//									  y_r  = y / 2 = 3
	//									  z_r  = z / 2 = 4 
	// 
	//  mentre i parametri x_i y_i_ z_i sono le componenti del punto di contatto posto sullo superficie
	//  espresso rispetto al sistema di riferimento posto al centro dell'oggetto
	//

	Eigen::MatrixXd R(3,3);

	double x_e = x_r - abs(x_i) ;
	double y_e = y_r - abs(y_i) ;
	double z_e = z_r - abs(z_i) ;

	// determino in che faccia della box si trova il punto di contatto passato alla funzine
	//
	if( x_e < y_e)
	{
		if(x_e < z_e)
		{
			if(x_i > 0)  
			{
				R << cos(90*PPI/180),  0, sin(90*PPI/180),
						       0   ,  1,    0 ,
					-sin(90*PPI/180), 0, cos(90*PPI/180);
			}
			else
			{	
				R << cos(-90*PPI/180),  0, sin(-90*PPI/180),
						       0    ,  1,    0 ,
					-sin(-90*PPI/180),  0, cos(-90*PPI/180);
			}
		}
		else
		{
			if(z_i > 0)
			{	
				R << cos(180*(PPI/180)),  0, sin(180*(PPI/180)),
					            	0   ,  1,    0 ,
					-sin(180*(PPI/180)), 0, cos(180*(PPI/180));
			}
			else    
			{
				R << 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1;
			}
		}
	}
	else
	{
		if(y_e < z_e)
		{
			if(y_i > 0)  // faccia 1
			{	
				R << 1, 0, 0,
					0, cos(-90*PPI/180), -sin(-90*PPI/180),
					0, sin(-90*PPI/180), cos(-90*PPI/180);
			}
			else
			{
				R << 1, 0, 0,
					0, cos(90*PPI/180), -sin(90*PPI/180),
					0, sin(90*(PPI/180)), cos(90*PPI/180);
			}
		}
		else
		{
			if(z_i > 0)
			{	
				R << cos(180*(PPI/180)),  0, sin(180*(PPI/180)),
						0   ,  1,    0 ,
					-sin(180*(PPI/180)), 0, cos(180*(PPI/180));
			}
			else
			{
				R << 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1;
			}
		}
	}


	// R_in = R.transpose();
	R_in = R;

	// c_R_o
}