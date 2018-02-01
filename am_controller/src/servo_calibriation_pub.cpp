#include "ros/ros.h"
#include "std_msgs/String.h"
#include "am_controller/servoset_srv.h"
#include <iostream>
using namespace std;


#include "ros/time.h"
#include <string>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <Eigen/Eigen> 


#define pi 3.141592653589793
using namespace Eigen;

geometry_msgs::PoseStamped base_pose;
int servo1=6;

void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	cout<<"enter pose_callback!"<<endl;
	
	Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
	Matrix3d Rw_three = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	/*Vector3d Ow_three(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
	Matrix4d Tw_three=MatrixXd::Zero(4,4);Tw_three(3,3)=1.0;
	Tw_three.block(0,0,3,3)=Rw_three;Tw_three.block(0,3,3,1)=Ow_three;

	Matrix4d Tthree_body=MatrixXd::Ones(4,4);
	Tthree_body.block(0,0,4,4)<<1,0,0,0,
								0,1,0,0,
								0,0,1,-0.484,
								0,0,0,1;
//	Tthree_body.block(3,0,1,3)<<0,0,0;
//	Tthree_body(0,3)=0;
//	Tthree_body(1,3)=0;
//	Tthree_body(2,3)=-0.484;

	Matrix4d Tw_body=Tw_three*Tthree_body;

	cout<<"position is:"<<endl;
	cout<<Ow_three<<endl;
	cout<<"rotation matrix is:"<<endl;
	cout<<Rw_three.matrix()<<endl;
	cout<<"body pose is :"<<endl;
	cout<<Tw_three.matrix()<<endl;
	cout<<"base pose is :"<<endl;
	cout<<Tw_body.matrix()<<endl;

	base_pose.header.stamp = msg->header.stamp;
	base_pose.header.frame_id = msg->header.frame_id;
	base_pose.pose.orientation.x = msg->pose.orientation.x;
	base_pose.pose.orientation.y = msg->pose.orientation.y;
	base_pose.pose.orientation.z = msg->pose.orientation.z;
	base_pose.pose.orientation.w = msg->pose.orientation.w;
	base_pose.pose.position.x = Tw_body(0,3);
	base_pose.pose.position.y = Tw_body(1,3);
	base_pose.pose.position.z = Tw_body(2,3);
	*/
	double roll, pitch, yaw;
//lib64	tf::Quaternion q;
//	tf::quaternionMsgToTF(msg->pose.orientation,q);
//	tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
Vector3d eular_angle = Rw_three.eulerAngles(2,1,0);
	cout.precision(4);
	cout<<"quaternion is:\n"<<q.coeffs()<<endl;
	cout<<"yaw pitch roll is\n"<<eular_angle*180/pi<<endl;
//	cout<<"pitch is:"<<pitch<<endl;

	ofstream foutC("/home/flx/servo.csv", ios::app);
	
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(3);
        foutC <<"output_angle"<< ","
              << eular_angle[1]*180/pi << ","
              << "input_angle" << ","
              <<servo1<< ","
              << "done" << endl;
        foutC.close();
	
	

}



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "servo_calibriation_pub");
	
	ros::NodeHandle n;
	ros::Subscriber pose_feedback=n.subscribe("/vrpn_client_node/three/pose",1,PoseStamped_Callback);
	//ros::Subscriber arm_pose_feedback=n.subscribe("/vrpn_client_node/four/pose",1,Arm_PoseStamped_Callback);
    ros::ServiceClient servo_calibriation = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
	am_controller::servoset_srv servo_msg;
	//ros::Rate loop_rate(10);

		for ( servo1 = 50; servo1 <= 140 ; servo1=servo1+1)
		{
			if(!ros::ok()){break;}
		//	for (int servo2 = 10; servo2 < 170; servo2=servo2+10)
		//	{
		//		if(!ros::ok()){break;}
		//		for (int servo3 = 10; servo3 < 170; servo3=servo3+10)
		//		{
				//	if(!ros::ok()){break;}
					 
					 servo_msg.request.cmd = 3;
					 servo_msg.request.pos1 = 95;
					 servo_msg.request.pos2 = 83;
					 servo_msg.request.pos3 = servo1;
					 servo_msg.request.pos4 = 22.19;
					 servo_msg.request.action_time = 1500;
					 servo_calibriation.call(servo_msg);
					 printf("set servo pose:%d	%d	%d	0.19\n",servo1,0,90);
					 ros::Duration(2).sleep();
					 ros::spinOnce();
					// while(!servo_calibriation.call(servo_msg)&&ros::ok()){}

					 //servo_msg.request.cmd = 1;
					// servo_calibriation.call(servo_msg);
					 if(servo_calibriation.call(servo_msg)){
						printf("get servo pose:%lf	%lf	%lf	%lf\n",servo_msg.response.servo_pos1,
																			servo_msg.response.servo_pos2,
																			servo_msg.response.servo_pos3,
																			servo_msg.response.servo_pos4);
					 }else{
						 ROS_ERROR("Fail to get the pose!!");
					 }
			//	}
				
		//	}
			
		}

//	ros::spin();
	return 0;
}