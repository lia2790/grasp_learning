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


double quality = 0;
//genero una indice di qualità casuale per ogni grasp di una box
//valori casuali da 0 a 100 (da offset a interval_size)


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

	ros::init(argc, argv, "Random_box");	// ROS node
	ros::NodeHandle nh;

	srand(time(NULL)); //se non voglio la stessa sequenza di numeri casuali

	file_output_comparison.open("box_db.csv", ofstream::app);


	box n_box[n_samples];  //array di box

    

	for(int i = 0; i < n_samples ; i++)  //per ogni box individuata dall'indice i-esimo vado a discretizzare la superficie
	{
        n_box[i].width  = (double)((rand() % interval_size) + offset ) / 100; // x
		n_box[i].height = (double)((rand() % interval_size) + offset ) / 100; // y
		n_box[i].length = (double)((rand() % interval_size) + offset ) / 100; // z
	
	Eigen::Vector3d axis_dimensions_box(3);

	axis_dimensions_box << n_box[i].width , n_box[i].height , n_box[i].length; // dimensioni della box
	int discrete_side = 2;         // in quante parti voglio discretizzare i lati della box
	double distance_hand = 0.05;   // 5 centimetri
	Eigen::Matrix4d T_fixed_frame(4,4);
	T_fixed_frame << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; // nessuna rotazione nessuna traslazione 
                                                                     // corrisponde al centro della box




  
	std::vector<Eigen::MatrixXd> grasp_point = populate_face(axis_dimensions_box, discrete_side, distance_hand, T_fixed_frame );
    //discretizzo le facce della i-esima box
    //ad ogni punto discretizzato corrisponde una trasformazione 

    

		for(int j = 0 ; j < grasp_point.size(); j++)
		{
            //quality = (double)((rand() % interval_size) + offset ) / 100; // probabilità di successo di grasp

            Matrix3f Rotation_matrix(3,3);
            Rotation_matrix << grasp_point[j](0,0) , grasp_point[j](0,1) , grasp_point[j](0,2),
                               grasp_point[j](1,0) , grasp_point[j](1,1) , grasp_point[j](1,2),
                               grasp_point[j](2,0) , grasp_point[j](2,1) , grasp_point[j](2,2);
            Quaternionf q(Rotation_matrix);

            q.normalize();




			file_output_comparison 
            <<n_box[i].width<<','<<n_box[i].height<<','<<n_box[i].length<<','                // box h l d
            <<grasp_point[j](0,3)<<','<<grasp_point[j](1,3)<<','<<grasp_point[j](2,3)<<','   // x y z
            <<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<endl;                                              // qx qy qz qw


            
          /*  <<quality
            <<' '<<"1:"<<n_box[i].width<<' '<<"2:"<<n_box[i].height<<' '<<"3:"<<n_box[i].length
            <<' '<<"4:"<<grasp_point[j](0,0)<<' '<<"5:"<<grasp_point[j](0,1)<<' '<<"6:"<<grasp_point[j](0,2)<<' '<<"7:"<<grasp_point[j](0,3)
            <<' '<<"8:"<<grasp_point[j](1,0)<<' '<<"9:"<<grasp_point[j](1,1)<<' '<<"10:"<<grasp_point[j](1,2)<<' '<<"11:"<<grasp_point[j](1,3)
            <<' '<<"12:"<<grasp_point[j](2,0)<<' '<<"13:"<<grasp_point[j](2,1)<<' '<<"14:"<<grasp_point[j](2,2)<<' '<<"15:"<<grasp_point[j](2,3)
            <<' '<<"16:"<<grasp_point[j](3,0)<<' '<<"17:"<<grasp_point[j](3,1)<<' '<<"18:"<<grasp_point[j](3,2)<<' '<<"19:"<<grasp_point[j](3,3)
            <<' '<<'\n'; */
           

        }
	}// end for


    file_output_comparison.close();


	cout << " STAMPATO " << endl;
    cout << "   fine   " << endl;


	ros::spin();
	return 0;
}
