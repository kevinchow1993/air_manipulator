#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>

double last_time;
double now;
double x_dot,y_dot,z_dot;
double last_x=0,last_y=0,last_z=0;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Localpose_Callback(const geometry_msgs::PoseStampedConstPtr & msg)
{	
	now = ros::Time::now().toSec();
	double dt = now-last_time;
	last_time=now;
	//if(dt>0.05){
		x_dot = (msg->pose.position.x-last_x)/dt;
		y_dot = (msg->pose.position.y-last_y)/dt;
		z_dot = (msg->pose.position.z-last_z)/dt;
		last_x = msg->pose.position.x;
		last_y = msg->pose.position.y;
		last_z = msg->pose.position.z;

		ROS_INFO("--X_dot:%.2f  --Y_dot:%.2f  --Z_dot:%.2f\n",x_dot,y_dot,z_dot);
	//}
	
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "uav_speed");
	ros::NodeHandle n;

	
	ros::Subscriber sub = n.subscribe("/mavros/vision_pose/pose", 1, Localpose_Callback);
	last_time = ros::Time::now().toSec();




	ros::spin();

	return 0;
}