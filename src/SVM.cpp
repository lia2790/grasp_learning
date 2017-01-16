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




int main (int argc, char **argv)
{

	//inizializzazione nodo 
	ros::init(argc, argv, "Grasp_poses");// ROS node
	ros::NodeHandle nh;


	




	ros::spin();
	return 0;
}
