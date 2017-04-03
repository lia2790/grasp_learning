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

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl_parser/kdl_parser.hpp>


#include <math.h>
#include <stdio.h>
#include <ctime>
#include <time.h>


using namespace std;
using namespace Eigen;
using namespace KDL;


inline bool CollisionAvoidance(Eigen::VectorXd &box, Eigen::VectorXd &cp, std::vector<Eigen::VectorXd> &hp)
{

	double box_x = box(0);
	double box_y = box(1);
	double box_z = box(2);

	double px = cp(0);
	double py = cp(1);
	double pz = cp(2)+box(2)/2;

	double qx = cp(3);
	double qy = cp(4);
	double qz = cp(5);
	double qw = cp(6);

	KDL::Vector t_cp(px,py,pz);
	KDL::Rotation R_cp = Rotation::Quaternion(qx,qy,qz,qw);

	KDL::Frame f_cp(R_cp,t_cp);


	for(int i = 0; i < hp.size(); i++)
	{
		KDL::Vector t_hp(hp[i](0), hp[i](1), hp[i](2));
		KDL::Rotation R_hp = Rotation::Quaternion(hp[i](3),hp[i](4),hp[i](5),hp[i](6));
		KDL::Frame f_hp(R_hp, t_hp);


		KDL::Frame f_cp_hp = f_cp*f_hp;

		if(f_cp_hp.p.z() < 0)
			return false;
	}	

	return true;
	
}