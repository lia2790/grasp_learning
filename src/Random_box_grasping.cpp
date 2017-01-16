#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>


#include <cmath>
#include <ctime>
#include <time.h>



using namespace std;
using namespace Eigen;



 
int n_samples = 1; //quante scatole voglio generare, ogni scatola è vista come un campione


struct box                  
{                       

    double width  = 0; // x
	double height = 0; // y
	double length = 0; // z

};
int offset = 1; // centimetri
int interval_size = 100; // centimetri
// intervallo di campionamento da 1cm (offset) a 1 m (interval_size)


ofstream file_output_comparison;












std::vector<Eigen::MatrixXd> populate_face(Eigen::Vector3d axis_dimensions, int disc, double dist_hand, Eigen::Matrix4d T_init)
{
    // std::vector<Eigen::Matrix4d> results((disc+1)*(disc+1)*6, Eigen::Matrix4d::Identity());
    /* The total size of the output vertor is disc*disc*(disc-)*6*/
    /* it is because each side of each box face is discretized by disc, then the angles are as well
    but PI an -PI is the same so the last is not evaluated, then there are 6 faces
    example if disc =3 ... 4*4*4*6    */


    std::vector<Eigen::MatrixXd> results;
    results.clear();


    Eigen::Matrix3d m, m_start;

    //  Plane YZ;
    for (double i = -1.0 * axis_dimensions(1) / 2 ; i <= axis_dimensions(1) / 2; i = i + (axis_dimensions(1) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(2) / 2 ; j <= axis_dimensions(2) / 2; j = j + (axis_dimensions(2) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = axis_dimensions(0) / 2 + dist_hand;
            start(1, 3) = i;
            start(2, 3) = j;
            m_start = Eigen::AngleAxisd( -M_PI / 2, Eigen::Vector3d::UnitY());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(0, 3) = -1.0 * (axis_dimensions(0) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitY());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }
    // Plane XZ;

    for (double i = -1.0 * axis_dimensions(0) / 2 ; i <= axis_dimensions(0) / 2; i = i + (axis_dimensions(0) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(2) / 2 ; j <= axis_dimensions(2) / 2; j = j + (axis_dimensions(2) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = i;
            start(1, 3) = axis_dimensions(1) / 2 + dist_hand;
            start(2, 3) = j;
            m_start = Eigen::AngleAxisd( M_PI / 2, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(1, 3) = -1.0 * (axis_dimensions(1) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( -M_PI / 2, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }

    // Plane XY

    for (double i = -1.0 * axis_dimensions(0) / 2 ; i <= axis_dimensions(0) / 2; i = i + (axis_dimensions(0) / disc) )
    {
        for (double j = -1.0 * axis_dimensions(1) / 2 ; j <= axis_dimensions(1) / 2; j = j + (axis_dimensions(1) / disc) )
        {
            Eigen::Matrix4d start = Eigen::MatrixXd::Identity(4, 4);

            start(0, 3) = i;
            start(1, 3) = j;
            start(2, 3) = axis_dimensions(2) / 2 + dist_hand;
            m_start = Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }

            start(2, 3) = -1.0 * (axis_dimensions(2) / 2 + dist_hand);
            m_start = Eigen::AngleAxisd( 0, Eigen::Vector3d::UnitX());
            for (double k = -M_PI ; k < M_PI; k = k + 2 * M_PI / disc)
            {
                m = m_start * Eigen::AngleAxisd(k, Eigen::Vector3d::UnitZ());
                start.block<3, 3>(0, 0) = m;
                results.push_back(T_init * start);
            }
        }
    }
    return results;

}















int main (int argc, char **argv)
{

	ros::init(argc, argv, "Random_box_grasping_quality_test");	// ROS node
	ros::NodeHandle nh;

	srand(time(NULL)); //se non voglio la stessa sequenza di numeri casuali

	file_output_comparison.open("data_set.txt", ofstream::app);


	box n_box[n_samples];  //array di box

    

	for(int n = 0; n < n_samples ; n++)  //per ogni box individuata dall'indice i-esimo vado a discretizzare la superficie
	{
        n_box[n].width  = (double)((rand() % interval_size) + offset ) / 100; // x
		n_box[n].height = (double)((rand() % interval_size) + offset ) / 100; // y
		n_box[n].length = (double)((rand() % interval_size) + offset ) / 100; // z
	
		Eigen::Vector3d axis_dimensions_box(3);

		axis_dimensions_box << n_box[n].width , n_box[n].height , n_box[n].length; // dimensioni della box
		int discrete_side = 2;         // in quante parti voglio discretizzare i lati della box
		double distance_hand = 0.05;   // 5 centimetri
		Eigen::Matrix4d T_fixed_frame(4,4);
		T_fixed_frame << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // nessuna rotazione nessuna traslazione 
                                                                     // corrisponde al centro della box

  
		std::vector<Eigen::MatrixXd> grasp_point = populate_face(axis_dimensions_box, discrete_side, distance_hand, T_fixed_frame );
    	//discretizzo le facce della i-esima box
    	//ad ogni punto discretizzato corrisponde una trasformazione 



	std::vector<Eigen::MatrixXd> Rotation ;

	std::vector<Eigen::VectorXd> Traslation ;

	std::vector<Eigen::MatrixXd> Grasp_Matrix_i ;

	std::vector<Eigen::VectorXd> Wrench_contact_i ;

	std::vector<Eigen::VectorXd> Wrench_soft_i ;	

	std::vector<double> Quality_i ;

	
	Eigen::MatrixXd Grasp_Matrix(6,6);
	Eigen::MatrixXd Skew_Matrix(3,3);
	Eigen::VectorXd Wrench_int(6);
	Eigen::VectorXd Wrench_ext(6);

	
	Eigen::MatrixXd Selection_Matrix_H(6,4);

	Selection_Matrix_H << 1, 0, 0, 0, 
						  0, 1, 0, 0, 
						  0, 0, 1, 0, 
						  0, 0, 0, 0,
						  0, 0, 0, 0,
						  0, 0, 0, 1; 


	Eigen::VectorXd Wrench_soft(4);
						
	Wrench_soft << 1 , 1 , 1 , 1;


	for(int i = 0 ; i < grasp_point.size(); i++) //grasp generati
	{
		Rotation.push_back(grasp_point[i].block<3,3>(0,0));
		Eigen::VectorXd dist(3);

		dist << grasp_point[i](0,3) , grasp_point[i](1,3) , grasp_point[i](2,3);

		Traslation.push_back(dist);



		Skew_Matrix(0,0) = Skew_Matrix(1,1) = Skew_Matrix(2,2) = 0;
		Skew_Matrix(0,1) = - grasp_point[i](2,3); // -rz
		Skew_Matrix(0,2) = grasp_point[i](1,3);   // ry
		Skew_Matrix(1,0) = grasp_point[i](2,3);   // rz
		Skew_Matrix(2,0) = - grasp_point[i](1,3); // -ry
		Skew_Matrix(1,2) = - grasp_point[i](0,3); // -rx
		Skew_Matrix(2,1) = grasp_point[i](0,3);   // rx



		Grasp_Matrix.block<3,3>(0,0) = Rotation[i];
		Grasp_Matrix.block<3,3>(3,3) = Rotation[i];
		Grasp_Matrix.block<3,3>(0,3) = Skew_Matrix * Rotation[i];
		Grasp_Matrix.block<3,3>(3,0) = MatrixXd::Zero(3,3);



	
		Grasp_Matrix_i.push_back(Grasp_Matrix);

		Wrench_soft_i.push_back(Wrench_soft);


	
		
		Wrench_contact_i.push_back(Selection_Matrix_H * Wrench_soft_i[i]);



		Wrench_int += ( - Grasp_Matrix_i[i] ) * Wrench_contact_i[i];


		



		JacobiSVD<MatrixXd> svd(Grasp_Matrix_i[i], ComputeThinU | ComputeThinV);

		Eigen::VectorXd Singular_Value_Grasp_Matrix(6);

		Singular_Value_Grasp_Matrix << 0, 0, 0, 0, 0, 0;

		Singular_Value_Grasp_Matrix = svd.singularValues();


		double sigma_min = Singular_Value_Grasp_Matrix[0];
		double sigma_max = Singular_Value_Grasp_Matrix[0];


		for(int i = 0 ; i < Singular_Value_Grasp_Matrix.size() ; i++)
		{
			if(Singular_Value_Grasp_Matrix[i] < sigma_min) sigma_min = Singular_Value_Grasp_Matrix[i];
			if(Singular_Value_Grasp_Matrix[i] > sigma_max) sigma_max = Singular_Value_Grasp_Matrix[i];
		}




		double quality_ = sigma_min / sigma_max ;

		Quality_i.push_back(quality_);



		cout<<"Singular_Value_Grasp_Matrix : "<< endl << Singular_Value_Grasp_Matrix << endl;
		cout<<"sigma_min :"<< endl << sigma_min << endl;
		cout<<"sigma_max :"<< endl << sigma_max << endl;
		cout<<"Quality = sigma_min / sigma_max :"<< endl << Quality_i[i] << endl;


		cout<<"Transform"<<endl;

		cout<<grasp_point[i](0,0)<<' '<<grasp_point[i](0,1)<<' '<<grasp_point[i](0,2)<<' '<<grasp_point[i](0,3)<<endl;
		cout<<grasp_point[i](1,0)<<' '<<grasp_point[i](1,1)<<' '<<grasp_point[i](1,2)<<' '<<grasp_point[i](1,3)<<endl;
		cout<<grasp_point[i](2,0)<<' '<<grasp_point[i](2,1)<<' '<<grasp_point[i](2,2)<<' '<<grasp_point[i](2,3)<<endl;
		cout<<grasp_point[i](3,0)<<' '<<grasp_point[i](3,1)<<' '<<grasp_point[i](3,2)<<' '<<grasp_point[i](3,3)<<endl;




		cout<<"Rotation"<<endl;

		cout<<Rotation[i](0,0)<<' '<<Rotation[i](0,1)<<' '<<Rotation[i](0,2)<<endl;
		cout<<Rotation[i](1,0)<<' '<<Rotation[i](1,1)<<' '<<Rotation[i](1,2)<<endl;
		cout<<Rotation[i](2,0)<<' '<<Rotation[i](2,1)<<' '<<Rotation[i](2,2)<<endl;



		cout<<"Traslation"<<endl;

		cout<<Traslation[i](0)<<' '<<Traslation[i](1)<<' '<<Traslation[i](2)<<endl;



		cout<<"Grasp_Matrix"<<endl;

		cout<<Grasp_Matrix_i[i](0,0)<<' '<<Grasp_Matrix_i[i](0,1)<<' '<<Grasp_Matrix_i[i](0,2)<<' '<<Grasp_Matrix_i[i](0,3)<<' '<<Grasp_Matrix_i[i](0,4)<<' '<<Grasp_Matrix_i[i](0,5)<<endl;
		cout<<Grasp_Matrix_i[i](1,0)<<' '<<Grasp_Matrix_i[i](1,1)<<' '<<Grasp_Matrix_i[i](1,2)<<' '<<Grasp_Matrix_i[i](1,3)<<' '<<Grasp_Matrix_i[i](1,4)<<' '<<Grasp_Matrix_i[i](1,5)<<endl;
		cout<<Grasp_Matrix_i[i](2,0)<<' '<<Grasp_Matrix_i[i](2,1)<<' '<<Grasp_Matrix_i[i](2,2)<<' '<<Grasp_Matrix_i[i](2,3)<<' '<<Grasp_Matrix_i[i](2,4)<<' '<<Grasp_Matrix_i[i](2,5)<<endl;
		cout<<Grasp_Matrix_i[i](3,0)<<' '<<Grasp_Matrix_i[i](3,1)<<' '<<Grasp_Matrix_i[i](3,2)<<' '<<Grasp_Matrix_i[i](3,3)<<' '<<Grasp_Matrix_i[i](3,4)<<' '<<Grasp_Matrix_i[i](3,5)<<endl;
		cout<<Grasp_Matrix_i[i](4,0)<<' '<<Grasp_Matrix_i[i](4,1)<<' '<<Grasp_Matrix_i[i](4,2)<<' '<<Grasp_Matrix_i[i](4,3)<<' '<<Grasp_Matrix_i[i](4,4)<<' '<<Grasp_Matrix_i[i](4,5)<<endl;
		cout<<Grasp_Matrix_i[i](5,0)<<' '<<Grasp_Matrix_i[i](5,1)<<' '<<Grasp_Matrix_i[i](5,2)<<' '<<Grasp_Matrix_i[i](5,3)<<' '<<Grasp_Matrix_i[i](5,4)<<' '<<Grasp_Matrix_i[i](5,5)<<endl;




			file_output_comparison
            <<Quality_i[i]
            <<' '<<"1:"<<n_box[n].width<<' '<<"2:"<<n_box[n].height<<' '<<"3:"<<n_box[n].length
            <<' '<<"4:"<<grasp_point[i](0,0)<<' '<<"5:"<<grasp_point[i](0,1)<<' '<<"6:"<<grasp_point[i](0,2)<<' '<<"7:"<<grasp_point[i](0,3)
            <<' '<<"8:"<<grasp_point[i](1,0)<<' '<<"9:"<<grasp_point[i](1,1)<<' '<<"10:"<<grasp_point[i](1,2)<<' '<<"11:"<<grasp_point[i](1,3)
            <<' '<<"12:"<<grasp_point[i](2,0)<<' '<<"13:"<<grasp_point[i](2,1)<<' '<<"14:"<<grasp_point[i](2,2)<<' '<<"15:"<<grasp_point[i](2,3)
            <<' '<<"16:"<<grasp_point[i](3,0)<<' '<<"17:"<<grasp_point[i](3,1)<<' '<<"18:"<<grasp_point[i](3,2)<<' '<<"19:"<<grasp_point[i](3,3)
            <<' '<<'\n'; /*
            <<n_box[i].width<<' '<<n_box[i].height<<' '<<n_box[i].length
            <<' '<<grasp_point[j](0,0)<<' '<<grasp_point[j](0,1)<<' '<<grasp_point[j](0,2)<<' '<<grasp_point[j](0,3)
            <<' '<<grasp_point[j](1,0)<<' '<<grasp_point[j](1,1)<<' '<<grasp_point[j](1,2)<<' '<<grasp_point[j](1,3)
            <<' '<<grasp_point[j](2,0)<<' '<<grasp_point[j](2,1)<<' '<<grasp_point[j](2,2)<<' '<<grasp_point[j](2,3)
            <<' '<<grasp_point[j](3,0)<<' '<<grasp_point[j](3,1)<<' '<<grasp_point[j](3,2)<<' '<<grasp_point[j](3,3)
            <<' '<<Quality_i[i]<<endl;*/
		
		}// end for grasp_point
	}// end box

	cout << " STAMPATO " << endl;
    cout << "   fine   " << endl;


	ros::spin();
	return 0;
}
