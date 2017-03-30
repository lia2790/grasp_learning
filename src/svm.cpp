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

	ros::init(argc, argv, "svm_predict");	// ROS node
	ros::NodeHandle nh;

	string relative_path_file_in;	
	string file_name_in;

	string relative_path_file_model;
	string file_name_model;

	string relative_path_file_out;
	string file_name_out;
	
	nh.param<std::string>("filename_in", relative_path_file_in, "/box_test/test" );
	nh.param<std::string>("filename_model", relative_path_file_model, "/svm_model/box1model");
	nh.param<std::string>("filename_out", relative_path_file_out, "/box_estimate/" );
	


	///////////////////// load the data_base ////////////////////////////////////
	std::string path = ros::package::getPath("grasp_learning");


	file_name_in = path + relative_path_file_in;//input box 
	ifstream file_in(file_name_in); 


	std::cout << "file: " << file_name_in.c_str() << " is " << (file_in.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_in.is_open())
	return 0;


	file_name_model = path + relative_path_file_model;//input model
	ifstream file_model(file_name_model);


	std::cout << "file: " << file_name_model.c_str() << " is " << (file_model.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_model.is_open())
	return 0;


   	ofstream file_output; //output file 
   	file_name_out = path + relative_path_file_out;
   	std::string name = "box_estimate";
    file_output.open( file_name_out + name, ofstream::app);




    int n_sv = 1;
    int dim_cols = 0;

    ///////////////////////////////// get values //////////////////////////////////
    std::string line_model; 
    getline( file_model, line_model, '\n' ); //count cols
    std::istringstream iss_line_model(line_model);
   	
    for(std::string value_model; getline(iss_line_model, value_model, ' ' ); )
    	dim_cols++;
	
    for(std::string line_model; getline( file_model, line_model, '\n' ); )
		n_sv++;
	////////////////////////////////////////////////////////////////////////////////

cout << "N_sv : " << n_sv << endl;
cout << "Dim_cols : " << dim_cols << endl;

	Eigen::MatrixXd Model = MatrixXd::Zero(n_sv,dim_cols);





	int n_samples = 1;
    int dim_sample = 0;

    ///////////////////////////////// get values //////////////////////////////////
    std::string line_in; 
    getline( file_in, line_in, '\n' ); //count cols
    std::istringstream iss_line_in(line_in);
  	
    for(std::string value_in; getline(iss_line_in, value_in, ' ' ); )
    	dim_sample++;
	
    for(std::string line; getline( file_in, line_in, '\n' ); )
		n_samples++;
	////////////////////////////////////////////////////////////////////////////////

cout << "n_samples : " << n_samples << endl;
cout << "dim_sample : " << dim_sample << endl;	






	file_name_model = path + relative_path_file_model;//input model
	ifstream filemodel(file_name_model);

	std::cout << "file: " << file_name_model.c_str() << " is " << (filemodel.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_model.is_open())
	return 0;

	///////////////////////////////// get values //////////////////////////////////
	int i=0;
	int j=0;
	for(std::string linemodel; getline(filemodel, linemodel, '\n' ); )
	{

		//cout << "yyy "<<endl;

    	std::istringstream iss_linemodel(linemodel);	
    	j=0;
    	for(std::string valuemodel; getline(iss_linemodel, valuemodel, ' ' ); )
    	{
    			//cout << " elem : " << stod(valuemodel) << endl;
    			Model(i,j) = stod(valuemodel);
    			j++;
    	}

    	i++;
    	
    }



	//cout << "Model : "<< endl << Model << endl;


    ifstream filein(file_name_in); 
	std::cout << "file: " << file_name_in.c_str() << " is " << (filein.is_open() == true ? "already" : "not") << " open" << std::endl;
	if(!file_in.is_open())
	return 0;



    Eigen::MatrixXd row = MatrixXd::Zero(1,dim_cols-1); // 11 elem
    Eigen::MatrixXd box_est = MatrixXd::Zero(n_samples,dim_cols-1);

	///////////////////////////////// get values //////////////////////////////////
	
	for(std::string line; getline( filein, line, '\n' ); )
	{
		std::vector<double> values_inline;
    	std::istringstream iss_line(line);	
    	for(std::string value; getline(iss_line, value, ' ' ); )
    		values_inline.push_back(stod(value));

    	Eigen::VectorXd X_sv(dim_cols-2);
    	Eigen::VectorXd X_test(dim_cols-2);	

    	for(int i = 0; i < (dim_cols-2); i++)
    		X_test(i) = values_inline[i];

    	Eigen::VectorXd dist = VectorXd::Zero(dim_cols-2);
    	Eigen::VectorXd arg(1);
    	arg << 0;
    	double y_est = 0;

    	for(int i = 0; i < n_sv ; i++)
    	{

    		for(int j=0 ; j< (dim_cols-2) ; j++)
    			X_sv(j) = Model(i,2+j);

    	
    		dist = X_sv - X_test;		

    		arg = dist.transpose() * dist;

    		double gamma = Model(i,0);

    		double weight = Model(i,1);

    		double expo = exp(-(gamma*arg(0)));

    		double gaussWeight = weight*expo;

    	
    		y_est = y_est + gaussWeight;

    		cout << "X_sv " << i << " : " << endl << X_sv.transpose() << endl;
    		cout << "X_test : " << endl << X_test.transpose() << endl;
    		cout << "dist  " << i << " : " <<endl<< dist.transpose() << endl;
    		cout << "arg  " << i << " : " << arg(0) << endl;
    		cout << "gamma " << i << "  : " << gamma << endl;
    		cout << "weight  " << i << " : " << weight << endl;
    		cout << "expo " << i << "  : " << expo << endl;
    		cout << "gaussWeight  " << i << " : " << gaussWeight << endl;
    		cout << "y_est " << i << " : " << y_est << endl;    		
    	}


    	cout << "y_est : " << y_est << endl;

    	row(0,0) = y_est;

    	for(int i = 0 ; i < (dim_cols-2) ; i++)
    		row(0,1+i) = X_test(i);

   
    	cout << " row : " << row << endl; 

    	int insert=0;
    	for(int i =0; i < n_samples; i++)
    	{
    		if(box_est(i,0) <= row(0,0))
    		{
    			insert = i;
    			break;
    		}	
    	}

    	int bloc = n_samples - insert - 1;
    	Eigen::MatrixXd app(bloc, 11);

		app = box_est.block(insert,0, bloc,11);    	

    	box_est.block(insert+1,0, bloc, 11) = app;
    	box_est.block<1,11>(insert,0) = row;
    }


    cout << " EST box : " << endl << box_est << endl;


    for(int i = 0 ; i < box_est.rows(); i++)
    {	
    	for(int j = 0 ; j < box_est.cols(); j++)
    		file_output<<box_est(i,j)<<' ';
   		file_output<<endl;
   	}


   	cout << "HAPPY" << endl;

    ros::spinOnce();
	return 0;

}