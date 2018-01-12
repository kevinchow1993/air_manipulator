#include "ros/ros.h"
#include "ros/time.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen> 


#define pi 3.141592653589793


using namespace std;
using namespace Eigen;
ros::Publisher body_puber;

void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
	Matrix3d Rw_three = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Ow_three(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Matrix4d Tw_three=MatrixXd::Zero(4,4);Tw_three(3,3)=1.0;
	Tw_three<<Rw_three;Tw_three.col(3)<<Ow_three;

	Matrix4d Tthree_body=MatrixXd::Ones(4,4);
	Tthree_body(0,3)=0;
	Tthree_body(1,3)=0;
	Tthree_body(2,3)=-0.25;

	Matrix4d Tw_body=Tw_three*Tthree_body;


	geometry_msgs::PoseStamped current_pose;
	current_pose.header.stamp = msg->header.stamp;
	current_pose.header.frame_id = msg->header.frame_id;
	current_pose.pose.orientation.x = msg->pose.orientation.x;
	current_pose.pose.orientation.y = msg->pose.orientation.y;
	current_pose.pose.orientation.z = msg->pose.orientation.z;
	current_pose.pose.orientation.w = msg->pose.orientation.w;
	current_pose.pose.position.x = Tw_body(0,3);
	current_pose.pose.position.y = Tw_body(1,3);
	current_pose.pose.position.z = Tw_body(2,3);

	body_puber.publish(current_pose);
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "uav_body_convert");
  	ros::NodeHandle n;


  	ros::Subscriber pose_feedback=n.subscribe("/vrpn_client_node/three/pose",1,PoseStamped_Callback);
  	body_puber = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
  	
  	ros::spin();

	return 0;

}