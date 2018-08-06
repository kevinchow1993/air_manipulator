#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include<fstream>  
#include "cmath"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "iostream"
#include <eigen_conversions/eigen_msg.h>//tf库中和eigen相互转换的库
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <camera_control/CameraPos.h>

#include "serial.h"


#define pi  3.1415926535898
using namespace std;

serial serial;//实例化一个串口类
//运动学模型的 endeffector系到camera 系的旋转，它们的原点重合
//
Eigen::Isometry3d Tbl,Tmb;
tf::Transform   camera_transform;
double Dpose=0;//相机关节运动学角度
int last_pose=58;
int flag_A=0,flag_B=0;

geometry_msgs::Transform target_pose,target_pose_inverse;
geometry_msgs::PointStamped target_point_world;

void PrintTransform(geometry_msgs::Transform T_msg){
	printf("translation:\nx:%f\ny:%f\nz:%f\nrotation:\nx:%f\ny:%f\nz:%f\nw:%f\n",T_msg.translation.x,T_msg.translation.y,T_msg.translation.z,
	T_msg.rotation.x,T_msg.rotation.y,T_msg.rotation.z,T_msg.rotation.w);
}

void set_TBH_by_Dpose()
{
	double cosb=cos(Dpose/180.0*pi);
	double sinb=sin(Dpose/180.0*pi);
	Tbl(0,0)=cosb;		Tbl(0,1)=0.0;		Tbl(0,2)=sinb;		Tbl(0,3)=0.2050;	//从body到link的偏移量  Tbl(0,3)=0.2050;
	Tbl(1,0)=0.0;		Tbl(1,1)=1.0;		Tbl(1,2)=0.0;		Tbl(1,3)=0.0;
	Tbl(2,0)=-sinb;		Tbl(2,1)=0.0;		Tbl(2,2)=cosb;		Tbl(2,3)=-0.062;//Tbl(2,3)=-0.062;
	Tbl(3,0)=0.0;		Tbl(3,1)=0.0;		Tbl(3,2)=0.0;		Tbl(3,3)=1.0;
}

void init_Tmb(void)
{
	//Tmb init
	Tmb=Eigen::Isometry3d::Identity();
	Tmb(0,0)=1.0;		Tmb(0,1)=0.0;		Tmb(0,2)=0.0;		Tmb(0,3)=0.0; //Tmb(0,3)=0.04;	//从manipulator到body
	Tmb(1,0)=0.0;		Tmb(1,1)=0.0;		Tmb(1,2)=1.0;		Tmb(1,3)=0.24;
	Tmb(2,0)=0.0;		Tmb(2,1)=-1.0;		Tmb(2,2)=0.0;		Tmb(2,3)=0.0;
	Tmb(3,0)=0.0;		Tmb(3,1)=0.0;		Tmb(3,2)=0.0;		Tmb(3,3)=1.0; 
}

void Set_Servo_Pos(int pos){
		if (pos>150)
		{
			pos = 150;
		}
		if (pos<43)
		{
			pos = 43;
		}
		char cmd[100];
		sprintf(cmd,"%ds",pos);//数字字符串化
		serial.Write(cmd,strlen(cmd));//向串口写入数据
		Dpose=-0.967742*pos+145.16129;
}




//相机的舵机 pose  在水平时150,往上增大，往下减小
//打算只set x ,y 方向的速度，先设置ｙ轴，使z轴保持在一定的高度，其中x轴和yaw角有关。
//我需要知道相机坐标系和机体坐标系之间的关系，以及apriltag和相机坐标系之间的关系
void CameraPos_CallBack(camera_control::CameraPos msg)
{
	Eigen::Isometry3d TCO_eigen;
	//tf::poseMsgToTF(msg.pose.pose,camera_transform);

	tf::poseMsgToEigen(msg.pose.pose,TCO_eigen);
	tf::transformEigenToTF(TCO_eigen,camera_transform);
	tf::transformEigenToMsg(TCO_eigen,target_pose);
	tf::transformEigenToMsg(TCO_eigen.inverse(),target_pose_inverse);
	flag_A ++;
	

	cout<<"----------------------"<<endl;
	cout<<"---camera_object---:\n"<<TCO_eigen.matrix()<<"\n---"<<endl;

	PrintTransform(target_pose);
	cout<<"---inverse---"<<endl;
	PrintTransform(target_pose_inverse);
	cout<<"----------------------"<<endl;
	

}



int main(int argc, char *argv[])
{
	

	ros::init(argc, argv, "hand_eye_calibrate");
	ros::NodeHandle n;
	std::string com_cs;

	ros::Publisher Base_hand_pub =n.advertise<geometry_msgs::Transform>("/world_effector",10);
	ros::Publisher camera_object_pub =n.advertise<geometry_msgs::Transform>("/camera_object",10);
	ros::Publisher target_point_pub = n.advertise<geometry_msgs::PointStamped>("/target_point",10); 
	ros::Subscriber Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,CameraPos_CallBack);//call by aprilTags pack.

	//ros::Subscriber pose_feedback=n.subscribe("/mavros/vision_pose/pose",1,vrpn_Callback);

	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
    serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);

	static tf::TransformBroadcaster br;
	static tf::TransformBroadcaster Tco_br;
	tf::TransformListener listener;

	bool flag = 0;
	ros::Duration(2.0).sleep();
	ros::Rate loop_rate(2);
	init_Tmb();


	while(ros::ok()){
		if(flag ==0){
			last_pose--;
			if(last_pose<50){
				flag=1;
			}
		}
		if(flag == 1){
			last_pose++;
			if(last_pose>80){
				flag=0;
			}
		}
		Set_Servo_Pos(last_pose);
		set_TBH_by_Dpose();
		loop_rate.sleep();
		//cout<<"--last_pose:"<<last_pose<<"---"<<"Dpose:"<<Dpose<<endl;
		tf::Transform transform;
		Eigen::Isometry3d Tml = Tmb*Tbl;
		geometry_msgs::Transform hand_pose;
		ros::spinOnce();

		

	 //	tf::poseEigenToMsg(Tml,hand_pose.pose);

	 


		if(flag_A==flag_B){
			ROS_ERROR("object is out of camera!!");
		}
		if(flag_A!=flag_B){


			tf::transformEigenToMsg(Tbl,hand_pose);
			tf::transformEigenToTF(Tbl,transform);
			cout<<"----------------------"<<endl;
			cout<<"---base_endeffector---\n"<<Tbl.matrix()<<endl;
			PrintTransform(hand_pose);
			cout<<"----------------------"<<endl;
			ros::Time Timenow= ros::Time::now();
			br.sendTransform(tf::StampedTransform(transform,Timenow,"base_link","end_effector"));
			
			Tco_br.sendTransform(tf::StampedTransform(camera_transform,Timenow,"camera_link","target_link"));
			camera_object_pub.publish(target_pose);
			Base_hand_pub.publish(hand_pose);
			

			ROS_INFO("%d data have been writen!",flag_A);
			flag_B=flag_A;
			//  camera_poses_timestamped.csv
			ofstream foutC("/home/flx/camera_poses_timestamped_kevin.csv", ios::app);
		    foutC.setf(ios::fixed, ios::floatfield);
			//foutC.precision(0);
			double now;
			now =ros::Time::now().toSec();
			foutC << now<< ", ";
		//	foutC.precision(9);
			foutC <<target_pose_inverse.translation.x<< ", "
				<< target_pose_inverse.translation.y << ", "
				<< target_pose_inverse.translation.z<< ", "
				<< target_pose_inverse.rotation.x << ", "
				<< target_pose_inverse.rotation.y << ", "
				<<target_pose_inverse.rotation.z << ", "
				<< target_pose_inverse.rotation.w << "\n";
			foutC.close();


			//B_H  
			ofstream foutBH("/home/flx/tf_poses_timestamped_ke.csv", ios::app);
		    foutBH.setf(ios::fixed, ios::floatfield);
			//foutBH.precision(0);
			foutBH << now << ", ";
		//	foutBH.precision(9);
			foutBH <<hand_pose.translation.x<< ", "
				<< hand_pose.translation.y << ", "
				<< hand_pose.translation.z<< ", "
				<< hand_pose.rotation.x << ", "
				<< hand_pose.rotation.y << ", "
				<<hand_pose.rotation.z << ", "
				<< hand_pose.rotation.w << endl;
			foutBH.close();

		}
		geometry_msgs::PointStamped target_point;
		target_point.header.frame_id = "target_link";
		target_point.point.x = 0;
		target_point.point.y = 0;
		target_point.point.z = 0;
		


		try
		{
			listener.transformPoint("base_link",target_point,target_point_world);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			continue;

		}
		tf::StampedTransform Tec;
		try
		{
			listener.lookupTransform("/end_effector","/camera_link",ros::Time(0),Tec);
			Eigen::Isometry3d tec_eigen;
			tf::transformTFToEigen(Tec,tec_eigen);
			cout<<tec_eigen.matrix()<<endl;

		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			continue;
		}
		
		target_point_pub.publish(target_point_world);

		
	}

	serial.Close();
	
	return 0;
}