#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cmath"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "iostream"
#include <eigen_conversions/eigen_msg.h>//tf库中和eigen相互转换的库
#include <camera_control/CameraPos.h>
#include <kindr/Core>//eth 的运动学动力学库，是基于Eigen的拓展
#include "serial.h"

#define pi  3.1415926535898
using namespace std;
Eigen::Vector3d t_cstar_o(0.0,0.0,0.0);

serial serial;//实例化一个串口类
//运动学模型的 endeffector系到camera 系的旋转，它们的原点重合
Eigen::Matrix3d R_e_c;
/*R_e_c <<0,0,1
		0,-1,0,
		1,0,0;*/
const double a1=0.2,a2=0.033;

double theta_1_dot,theta_2_dot;
double x_dt,y_dt,z_dt,wx,wy,wz;//camera 速度
Eigen::Vector3d p_dt;//相机线速度
Eigen::Vector3d w;//相机角速度

double Dpose;//相机关节运动学角度
double  theta1=0.5;

double  theta2; //关节角度
int last_pose=52;




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
}




//相机的舵机 pose  在水平时150,往上增大，往下减小
//打算只set x ,y 方向的速度，先设置ｙ轴，使z轴保持在一定的高度，其中x轴和yaw角有关。
//我需要知道相机坐标系和机体坐标系之间的关系，以及apriltag和相机坐标系之间的关系
void set_servo_vel(Eigen::Vector3d vel_l,Eigen::Vector3d vel_w){

/*R_e_c <<0,0,1
		0,-1,0,
		1,0,0;*/
		x_dt =  vel_l(2);
		y_dt = -vel_l(1);
		z_dt =  vel_l(0);
		wx   =  vel_w(2);
		wy   = -vel_w(1);
		wz   =  vel_w(0);
		//cout<<"x,y,z,wx,wy,wz\n"<<x_dt<<"\n"<<y_dt<<"\n"<<z_dt<<"\n"<<wx<<"\n"<<wy<<"\n"<<wz<<endl;
		//cout<<"atan theta1="<<atan(-wx/wy)<<endl;
		//theta1 = 
		theta2 = last_pose*pi/180;
		theta_2_dot = -x_dt/(a2*sin(theta2));
		//theta_2_dot = z_dt/(a2*cos(theta2));
		//theta_1_dot = -(x_dt+a1*cos(theta1)*sin(theta2)*theta_2_dot)/(sin(theta1)*(a1+a2*cos(theta2)));
		//theta_1_dot=(y_dt*cos(theta1))/(a1*pow(sin(theta1),2) + a1*pow(cos(theta1),2) + a2*pow(cos(theta1),2)*cos(theta2) + a2*cos(theta2)*pow(sin(theta1),2)) 
		//			- (x_dt*sin(theta1))/(a1*pow(sin(theta1),2) + a1*pow(cos(theta1),2) + a2*pow(cos(theta1),2)*cos(theta2) + a2*cos(theta2)*pow(sin(theta1),2));
		

		//cout<<"theta1= "<<theta1<<endl;
		//cout<<"theta2= "<<theta2<<"\t"<<z_dt/(a2*theta_2_dot)<<endl;
		//cout<<"theta_2_v2 =" <<(a2*theta_2_dot*sin(theta1)*sin(theta2))/(a1*cos(theta1) + a2*cos(theta1)*cos(theta2))<<endl;
		cout<<"theta_1_dot="<<theta_1_dot<<endl;
		cout<<"theta_2_dot="<<theta_2_dot<<endl;
		cout<<"theta_2_dotV2:"<<-(x_dt*cos(theta1))/(a2*sin(theta2)*pow(cos(theta1),2) + a2*sin(theta2)*pow(sin(theta1),2)) - (y_dt*sin(theta1))/(a2*sin(theta2)*pow(cos(theta1),2) + a2*sin(theta2)*pow(sin(theta1),2))<<endl;
		if (fabs(theta_2_dot*0.2)<20){
			last_pose = last_pose - (int)(theta_2_dot*180*0.1/3.14);
		}
	
		Set_Servo_Pos(last_pose);
		ros::Duration(0.2).sleep(); 
		cout<<"---\nnow set pose:"<<last_pose<<"\n---"<<endl;
		

		
		




}

void CameraPos_CallBack(camera_control::CameraPos msg)
{
	Eigen::Vector3d t_c_o;
	double marker_theta;
	Eigen::Vector3d marker_U;
	Eigen::Quaterniond marker_quaternion;
	Eigen::AngleAxisd marker_theta_U;
	double lamda=0.1;
	Eigen::Vector3d servo_vel_l;//线速度
	Eigen::Vector3d servo_vel_w;//角速度



	if(msg.id = 25){
		tf::pointMsgToEigen(msg.pose.pose.position,t_c_o);
		tf::quaternionMsgToEigen(msg.pose.pose.orientation,marker_quaternion);
		
		kindr::RotationQuaternionD kindr_quat(marker_quaternion);
		kindr::AngleAxisD kindr_theta_U(kindr_quat);
		marker_theta = kindr_theta_U.angle();
		marker_U = kindr_theta_U.axis();
	//	cout<<"kindr_theta_U is :\n"<<kindr_theta_U<<endl;
		marker_theta_U = kindr_theta_U.toImplementation();
		cout<<"eigen marker_theta is:\n"<<marker_theta_U.angle()<<"\neigen marker_U is:\n"<<marker_theta_U.axis()<<endl;
		servo_vel_l = -lamda*((t_cstar_o-t_c_o)+t_c_o.cross(marker_theta*marker_U));
		servo_vel_w = -lamda*marker_theta*marker_U;
		cout<<"servo_vel_l is:\n"<<servo_vel_l<<endl;
		cout<<"servo_vel_w is:\n"<<servo_vel_w<<endl;
		set_servo_vel(servo_vel_l,servo_vel_w);
	


	}
}

int main(int argc, char *argv[])
{
	

	ros::init(argc, argv, "pbvs");
	ros::NodeHandle n;
	std::string com_cs;
	ros::Subscriber Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,CameraPos_CallBack);//call by aprilTags pack.
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	//serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);
	Set_Servo_Pos(last_pose);

	//ros::spin();
	//serial.Close();
	return 0;
}