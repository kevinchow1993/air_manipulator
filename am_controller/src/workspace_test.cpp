#include "ros/ros.h"
#include <stdio.h>
#include "Kin.h"


#include <iostream>
#include <fstream>

bool  Valid_Angle(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4)//360
{
		double servo_pos1=5.0-joint_pos1;
		double servo_pos2=1.2*joint_pos2-25.0;
		double servo_pos3=-1.1*joint_pos2 -0.64444*joint_pos3+207;
		double servo_pos4=joint_pos4+2;
		if (servo_pos1<0||servo_pos2<0||servo_pos3<0||servo_pos4<0||servo_pos1>180||servo_pos2>180||servo_pos3>180||servo_pos4>180)
		{
			return false;
		}
		else return true;
}
int Valid_Space(FLX_kinematics &Kin)
{
		for (int i = 0; i < 2; ++i)
		{
			double joint_pos1=Kin.theta[i].theta1*180.0/pi;
			double joint_pos2=Kin.theta[i].theta2*180.0/pi;
			double joint_pos3=Kin.theta[i].theta3*180.0/pi;
			double joint_pos4=Kin.theta[i].theta4*180.0/pi;
			// printf("test:%lf %lf %lf %lf\n",joint_pos1,joint_pos2,joint_pos3,joint_pos4 );
			if (
					(Kin.theta[i].theta1<5.0*pi/180.0)&&(Kin.theta[i].theta1>-150.0*pi/180.0)&&
					(Kin.theta[i].theta2>0)&&(Kin.theta[i].theta2<pi)//&&
					//(Kin->theta[i].theta3)&&(Kin->theta[i].theta3)
			)return i+1;
		}
		return 0;
}
bool get_Kinematic_valid(double theta_e,double theta4,double px,double py,double &theta1,double &theta2,double &theta3)
{
		FLX_kinematics Kin; 
		Kin.Inverse_Kinematics(	theta_e,theta4,px,py);
		bool solution_exist=true;
		if (Kin.theta.size())
		{
			int valid_n=Valid_Space(Kin);
			if (valid_n>0)
			{
				//Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
				double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
				joint_pos1=Kin.theta[valid_n-1].theta1*180.0/pi;joint_pos2=Kin.theta[valid_n-1].theta2*180.0/pi;joint_pos3=Kin.theta[valid_n-1].theta3*180.0/pi;joint_pos4=Kin.theta[valid_n-1].theta4*180.0/pi;
				while(joint_pos1>180)joint_pos1-=180;while(joint_pos2>180)joint_pos2-=180;while(joint_pos3>180)joint_pos3-=180;while(joint_pos4>180)joint_pos4-=180;
				if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
				{
					theta1=joint_pos1*pi/180.0;theta2=joint_pos2*pi/180.0;theta3=joint_pos3*pi/180.0;
				}
				else solution_exist=false;
				
			}
			else solution_exist=false;
		}
		else solution_exist=false;
		return solution_exist;
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"workspace_test");
	ros::NodeHandle n;
	
	std::ofstream workspace_data;
	workspace_data.open("workspace.out" , ios::out | ios::trunc);
	double x_s=-0.5,x_t=0.5;
	double y_s=-0.5,y_t=0.5;
	double step=0.001;
	double theta1,theta2,theta3;
	double theta_e=0;
	double theta4=0;
	for(double x=x_s;x<x_t;x+=step)
	for(double y=y_s;y<y_t;y+=step)
	{
		if(get_Kinematic_valid(theta_e,theta4,x,y,theta1,theta2,theta3))
		workspace_data<<theta1<<" "<<theta2<<" "<<theta3<<" "<<x<<" "<<y<<endl;
	}
	workspace_data.close();

	// FLX_kinematics Kin; 
	// Kin.Forward_Kinematics(pi/4,-pi/5,pi/3,-pi/3);
	// Kin.Show_Forward_Result_RT();

	system("python ~/catkin_al/workspace_plot.py");

	return 0;

}