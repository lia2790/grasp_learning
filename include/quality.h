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

#include "pseudo_inverse.h"

using namespace Eigen;
using namespace std;

inline double quality(int which_quality, Eigen::MatrixXd &Grasp_Matrix_Contact, Eigen::MatrixXd &Hand_Jacobian_Contact)
{
	Eigen::VectorXd Singular ;
	double k = 1 ;
    double sigma_min ;
	double sigma_max ;
	double quality ;
	int nq_hand = Hand_Jacobian_Contact.cols();
      


	// GRASP JACOBIAN
    Eigen::MatrixXd GRASP_Jacobian(6,nq_hand-1); // H = (G+)^t * J
    Eigen::MatrixXd Grasp_Matrix_pseudo(Grasp_Matrix_Contact.rows(),Grasp_Matrix_Contact.cols()) ;
    pseudo_inverse(Grasp_Matrix_Contact,Grasp_Matrix_pseudo);

    GRASP_Jacobian = Grasp_Matrix_pseudo.transpose() * Hand_Jacobian_Contact; // (G+)^t * J



	cout << "°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°" << endl;
	cout << endl;
	cout << "	       INSIDE quality_ROA		           " << endl;
	cout << endl;
	cout << "°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°°" << endl;




	switch(which_quality) 
    {
        
        case 0: // "minimum_singular_value_of_G"  Q = sigma_min(G)
			{
        	   	JacobiSVD<MatrixXd> svd0(Grasp_Matrix_Contact, ComputeThinU | ComputeThinV);  
        		Singular = svd0.singularValues();

        		cout << "Singular : " << endl << Singular << endl;
        		cout << " Singular.size () : " << endl << Singular[Singular.size()-1] << endl;

        		double res = Singular[Singular.size()-1];

				return res;
			} 
        	break;


        case 1: // "Volume of the ellipsoid in the wrench space"  Q = K sqrt(det(GG.t)) = k ( sigma_0 **** sigma_d)
        	{
                Eigen::MatrixXd G_G_t = Grasp_Matrix_Contact * Grasp_Matrix_Contact.transpose();
				JacobiSVD<MatrixXd> svd1(G_G_t, ComputeThinU | ComputeThinV);  
				Singular = svd1.singularValues();

				cout << "Singular : " << endl << Singular << endl;
        		
        		quality = 1;

				for(int i = 0 ; i < Singular.size() ; i++)
					quality = quality * Singular[i];


				cout << "quality : " << endl << quality << endl;

				return ( quality * k );
			}
        	break;


        case 2: // "Grasp isotropy index" Q = sigma_min(G) / sigma_max(G) 
        	{
        		JacobiSVD<MatrixXd> svd2(Grasp_Matrix_Contact, ComputeThinU | ComputeThinV);  
				Singular = svd2.singularValues();
                sigma_min = Singular[Singular.size()-1];
				sigma_max = Singular[0];


				cout << "Singular : " << endl << Singular << endl;
				cout << "sigma_min : " << endl << sigma_min << endl;
				cout << " sigma_max : " << endl << sigma_max << endl;

				double res = sigma_min/sigma_max;

				return res;
			}
			break;



		case 3: // "Distance to singular configuration" Q = sigma_min(H) H = G.pseudo_inverse.transpose * J
			{
				JacobiSVD<MatrixXd> svd3(GRASP_Jacobian, ComputeThinU | ComputeThinV);  
				Singular = svd3.singularValues();
            
              	return Singular[Singular.size()-1];
			}
			break;



		case 4: // "Volume of manipulability ellipsoid" Q = K sqrt(det(HH.t))
			{
				Eigen::MatrixXd H_H_t = GRASP_Jacobian * GRASP_Jacobian.transpose();
				JacobiSVD<MatrixXd> svd4(H_H_t, ComputeThinU | ComputeThinV);  

        		
				Singular = svd4.singularValues();

					
				for(int i = 0 ; i < Singular.size() ; i++)
					quality *= Singular[i];


				return ( quality * k );
			}
			break;


		case 5: // "Uniformity of transformations" Q = sigma_min(H) / sigma_max(H) 
			{
				JacobiSVD<MatrixXd> svd5(GRASP_Jacobian, ComputeThinU | ComputeThinV);  

        			
				Singular = svd5.singularValues();

				sigma_min = Singular[Singular.size()-1];
				sigma_max = Singular[0];

				return sigma_min/sigma_max;
			}
			break;


		case -1: 
				return 8888; // no contact
				break;

      }//end switch

}