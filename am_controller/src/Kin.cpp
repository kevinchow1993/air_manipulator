#include "Kin.h"


FLX_kinematics::FLX_kinematics()
{

}

FLX_kinematics::~FLX_kinematics()
{

}

void FLX_kinematics::Forward_Kinematics(double angle1,double angle2,double angle3,double angle4)
{
			r11=cos(angle1 + angle2 + angle3)*cos(angle4);r12=-cos(angle1 + angle2 + angle3)*sin(angle4);r13=sin(angle1 + angle2 + angle3);
			r21=sin(angle1 + angle2 + angle3)*cos(angle4);r22=-sin(angle1 + angle2 + angle3)*sin(angle4);r23=-cos(angle1 + angle2 + angle3);
			r31=sin(angle4);r32=cos(angle4);r33=0.0;
			t1=(11.0*cos(angle1 + angle2))/100.0 + (179.0*cos(angle1))/1000.0 + (13.0*sqrt(101.0)*cos(angle1 + angle2 + angle3 - atan(10.0)))/1000.0;
			t2=(11.0*sin(angle1 + angle2))/100.0 + (179.0*sin(angle1))/1000.0 - (13.0*sqrt(101.0)*cos(angle1 + angle2 + angle3 + atan(1.0/10.0)))/1000.0;
			t3=0.0;
}

void FLX_kinematics::Forward_Kinematics(double angle1,double angle2,double angle3,double angle4,Matrix4d &result_SE3)
{
			r11=cos(angle1 + angle2 + angle3)*cos(angle4);r12=-cos(angle1 + angle2 + angle3)*sin(angle4);r13=sin(angle1 + angle2 + angle3);
			r21=sin(angle1 + angle2 + angle3)*cos(angle4);r22=-sin(angle1 + angle2 + angle3)*sin(angle4);r23=-cos(angle1 + angle2 + angle3);
			r31=sin(angle4);r32=cos(angle4);r33=0.0;
			t1=(11.0*cos(angle1 + angle2))/100.0 + (179.0*cos(angle1))/1000.0 + (13.0*sqrt(101.0)*cos(angle1 + angle2 + angle3 - atan(10.0)))/1000.0;
			t2=(11.0*sin(angle1 + angle2))/100.0 + (179.0*sin(angle1))/1000.0 - (13.0*sqrt(101.0)*cos(angle1 + angle2 + angle3 + atan(1.0/10.0)))/1000.0;
			t3=0.0;

			result_SE3(0,0)=r11;		result_SE3(0,1)=r12;		result_SE3(0,2)=r13;		result_SE3(0,3)=t1;
			result_SE3(1,0)=r21;		result_SE3(1,1)=r22;		result_SE3(1,2)=r23;		result_SE3(1,3)=t2;
			result_SE3(2,0)=r31;		result_SE3(2,1)=r32;		result_SE3(2,2)=r33;		result_SE3(2,3)=t3;
			result_SE3(3,0)=0.0;		result_SE3(3,1)=0.0;		result_SE3(3,2)=0.0;		result_SE3(3,3)=1.0;

}

void FLX_kinematics::Forward_Link_Kinematics(int links,double angle1,double angle2,double angle3,double angle4)
{
			switch(links)
			{
				case 1:
					r11=cos(angle1);r12=-sin(angle1);r13=0.0;
					r21=sin(angle1);r22=cos(angle1);r23=0.0; 
					r31=0.0;r32=0.0;r33=1.0;
					t1=(179.0*cos(angle1))/1000.0;
					t2=(179.0*sin(angle1))/1000.0;
					t3=0.0;
					break;
				case 2:
					r11=cos(angle1 + angle2);r12=-sin(angle1 + angle2);r13=0.0; 
					r21=sin(angle1 + angle2),r22=cos(angle1 + angle2);r23=0.0; 
					r31=0.0;r32=0.0;r33=1.0;
					t1=(11.0*cos(angle1 + angle2))/100.0 + (179.0*cos(angle1))/1000.0;
					t2=(11.0*sin(angle1 + angle2))/100.0 + (179.0*sin(angle1))/1000.0;
					t3=0.0;
					break;
				case 3:
					r11=cos(angle1 + angle2 + angle3);r12= 0.0;r13=sin(angle1 + angle2 + angle3);
					r21=sin(angle1 + angle2 + angle3);r22= 0.0;r23=-cos(angle1 + angle2 + angle3);
					r31=0.0;r32=1.0;r33=0.0;
					t1=(13.0*cos(angle1 + angle2 + angle3))/1000.0 + (11.0*cos(angle1 + angle2))/100.0 + (179.0*cos(angle1))/1000.0;
					t2=(13.0*sin(angle1 + angle2 + angle3))/1000.0 + (11.0*sin(angle1 + angle2))/100.0 + (179.0*sin(angle1))/1000.0;
					t3=0.0;
					break;
				case 4:
					Forward_Kinematics(angle1,angle2,angle3,angle4);
					break;
			}
}

void FLX_kinematics::Show_Forward_Result_RT(void)
{
			printf("Forward_Result_RT:\n");
			printf("%10f %10f %10f %10f\n",r11,r12,r13,t1);
			printf("%10f %10f %10f %10f\n",r21,r22,r23,t2);
			printf("%10f %10f %10f %10f\n",r31,r32,r33,t3);
}
void FLX_kinematics::Inverse_Kinematics(double nx0,double ox0,double ax0,double px0,
								double ny0,double oy0,double ay0,double py0,
								double nz0,double oz0,double az0,double pz0)
{
			theta.clear();
			nx= nx0;ox= ox0;ax= ax0;px= px0;
			ny= ny0;oy= oy0;ay= ay0;py= py0;
			nz= nz0;oz= oz0;az= az0;pz= pz0;
			double theta1_a,theta1_b,theta2,theta3,theta4;

			theta4=atan2(nz,oz);
			double theta123,theta12_a,theta12_b;
			theta123=atan2(ax,-ay);


			double a,b,an1,an2,m,n,u1,u2;
			a=110.0/1000;b=179.0/1000;
			u1=atan(10.0);u2=atan(1.0/10.0);
			an1=(13.0*sqrt(101.0)*(-ay*cos(u1)+ax*sin(u1)))/1000.0;//an1=(13.0*sqrt(101.0)*cos(theta123 - atan(10.0)))/1000.0;
			an2=(13.0*sqrt(101.0)*(-ay*cos(u2)-ax*sin(u2)))/1000.0;//an2=(13.0*sqrt(101.0)*cos(theta123 + atan(1.0/10.0)))/1000.0;
			n=px-an1;
			m=py+an2;

			double s,squ_s,den1,den2;
			s=(- a*a + 2*a*b - b*b + m*m + n*n)*(a*a + 2*a*b + b*b - m*m - n*n);
			if (s<0)
			{
				if (s>-1e-15)s=0;
				else return;
				
			}
			squ_s=sqrt(s);
			den1=(- a*a + b*b + 2*b*n + m*m + n*n);
			den2=(a*a + 2*a*n - b*b + m*m + n*n);


			theta12_a=	-2.0*atan2(((b*b*(2*b*m + squ_s))/den1 - 2*b*m - (a*a*(2*b*m + squ_s))/den1 - 2*a*m + (m*m*(2*b*m + squ_s))/den1 + (n*n*(2*b*m + squ_s))/den1 + (2*b*n*(2*b*m + squ_s))/den1),den2);
			theta12_b=	-2.0*atan2(((b*b*(2*b*m - squ_s))/den1 - 2*b*m - (a*a*(2*b*m - squ_s))/den1 - 2*a*m + (m*m*(2*b*m - squ_s))/den1 + (n*n*(2*b*m - squ_s))/den1 + (2*b*n*(2*b*m - squ_s))/den1),den2);     

			theta1_a=	2.0*atan2((2*b*m + squ_s),den1);
			theta1_b=	2.0*atan2((2*b*m - squ_s),den1);



			//theta2=theta12-theta1;
			//theta3=theta123-theta12;

			Theta theta_result;
			theta_result.theta1=theta1_a;
			theta_result.theta2=theta12_a-theta_result.theta1;
			theta_result.theta3=theta123-theta12_a;
			theta_result.theta4=theta4;
			theta.push_back(theta_result);


			theta_result.theta1=theta1_b;
			theta_result.theta2=theta12_b-theta_result.theta1;
			theta_result.theta3=theta123-theta12_b;
			theta_result.theta4=theta4;
			theta.push_back(theta_result);


}
void FLX_kinematics::Inverse_Kinematics(double theta_e,double theta4,double px,double py)
{
			theta.clear();
			double theta1_a,theta1_b,theta2,theta3;
			double theta123,theta12_a,theta12_b;
			theta123=pi/2.0+theta_e;


			double a,b,an1,an2,m,n,u1,u2;
			a=110.0/1000;b=179.0/1000;
			u1=atan(10.0);u2=atan(1.0/10.0);
			an1=(13.0*sqrt(101.0)*cos(theta123 - u1))/1000.0;
			an2=(13.0*sqrt(101.0)*cos(theta123 + u2))/1000.0;
			n=px-an1;
			m=py+an2;

			double s,squ_s,den1,den2;
			s=(- a*a + 2*a*b - b*b + m*m + n*n)*(a*a + 2*a*b + b*b - m*m - n*n);
			if (s<0)
			{
				if (s>-1e-15)s=0;
				else return;

			}
			squ_s=sqrt(s);
			den1=(- a*a + b*b + 2*b*n + m*m + n*n);
			den2=(a*a + 2*a*n - b*b + m*m + n*n);


			theta12_a=	-2.0*atan2(((b*b*(2*b*m + squ_s))/den1 - 2*b*m - (a*a*(2*b*m + squ_s))/den1 - 2*a*m + (m*m*(2*b*m + squ_s))/den1 + (n*n*(2*b*m + squ_s))/den1 + (2*b*n*(2*b*m + squ_s))/den1),den2);
			theta12_b=	-2.0*atan2(((b*b*(2*b*m - squ_s))/den1 - 2*b*m - (a*a*(2*b*m - squ_s))/den1 - 2*a*m + (m*m*(2*b*m - squ_s))/den1 + (n*n*(2*b*m - squ_s))/den1 + (2*b*n*(2*b*m - squ_s))/den1),den2);     
			theta1_a=	2.0*atan2((2*b*m + squ_s),den1);
			theta1_b=	2.0*atan2((2*b*m - squ_s),den1);
			//theta2=theta12-theta1;
			//theta3=theta123-theta12;
			Theta theta_result;
			theta_result.theta1=theta1_a;
			theta_result.theta2=theta12_a-theta_result.theta1;
			theta_result.theta3=theta123-theta12_a;
			theta_result.theta4=theta4;
			Form_result(theta_result);
			theta.push_back(theta_result);


			theta_result.theta1=theta1_b;
			theta_result.theta2=theta12_b-theta_result.theta1;
			theta_result.theta3=theta123-theta12_b;
			theta_result.theta4=theta4;
			Form_result(theta_result);
			theta.push_back(theta_result);
}

double FLX_kinematics::Form_postive_negative_180(double ang)
{
	while(ang>=pi)ang-=(2*pi);
	while(ang<-pi)ang+=(2*pi);
	return ang;
}

void FLX_kinematics::Form_result(Theta &theta_result)
{
	theta_result.theta1=Form_postive_negative_180(theta_result.theta1);
	theta_result.theta2=Form_postive_negative_180(theta_result.theta2);
	theta_result.theta3=Form_postive_negative_180(theta_result.theta3);
	theta_result.theta4=Form_postive_negative_180(theta_result.theta4);
}

void FLX_kinematics::Show_Inverse_Result_Theta(Theta theta_result)
{
	
			printf("\nInverse_Result:\n");
			printf("Rad: %10f %10f %10f %10f\n",theta_result.theta1,theta_result.theta2,theta_result.theta3,theta_result.theta4);
			printf("Ang: %10f %10f %10f %10f\n",theta_result.theta1*180.0/pi,theta_result.theta2*180.0/pi,theta_result.theta3*180.0/pi,theta_result.theta4*180.0/pi);

			Forward_Kinematics(theta_result.theta1,theta_result.theta2,theta_result.theta3,theta_result.theta4);
			Show_Forward_Result_RT();

}


bool  FLX_kinematics::Valid_Angle(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4)//360
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

int FLX_kinematics::Valid_Space()
{
		for (int i = 0; i < 2; ++i)
		{
			if (
					(theta[i].theta1<5.0*pi/180.0)&&(theta[i].theta1>-150.0*pi/180.0)&&
					(theta[i].theta2>0)&&(theta[i].theta2<pi)//&&
					//(Kin->theta[i].theta3)&&(Kin->theta[i].theta3)
			)return i+1;
		}
		return 0;
}

bool FLX_kinematics::Check_valid(double theta_e,double theta4,double px,double py,double &theta1_,double &theta2_,double &theta3_)
{
	Inverse_Kinematics(	theta_e,theta4,px,py);
	bool solution_exist=true;
	if (theta.size())
	{
			int valid_n=Valid_Space();
			if (valid_n>0)
			{
				//Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
				double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
				joint_pos1=theta[valid_n-1].theta1*180.0/pi;joint_pos2=theta[valid_n-1].theta2*180.0/pi;joint_pos3=theta[valid_n-1].theta3*180.0/pi;joint_pos4=theta[valid_n-1].theta4*180.0/pi;
				while(joint_pos1>180)joint_pos1-=180;while(joint_pos2>180)joint_pos2-=180;while(joint_pos3>180)joint_pos3-=180;while(joint_pos4>180)joint_pos4-=180;
				if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
				{
					theta1_=joint_pos1*pi/180.0;theta2_=joint_pos2*pi/180.0;theta3_=joint_pos3*pi/180.0;
				}
				else solution_exist=false;
				
			}
			else solution_exist=false;
	}
	else solution_exist=false;
	return solution_exist;
}
bool FLX_kinematics::Check_valid(double theta_e,double theta4,double px,double py)
{
	Inverse_Kinematics(	theta_e,theta4,px,py);
	bool solution_exist=true;
	if (theta.size())
	{
			int valid_n=Valid_Space();
			if (valid_n>0)
			{
				//Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
				double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
				joint_pos1=theta[valid_n-1].theta1*180.0/pi;joint_pos2=theta[valid_n-1].theta2*180.0/pi;joint_pos3=theta[valid_n-1].theta3*180.0/pi;joint_pos4=theta[valid_n-1].theta4*180.0/pi;
				while(joint_pos1>180)joint_pos1-=180;while(joint_pos2>180)joint_pos2-=180;while(joint_pos3>180)joint_pos3-=180;while(joint_pos4>180)joint_pos4-=180;
				if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
				{
					//theta1_=joint_pos1*pi/180.0;theta2_=joint_pos2*pi/180.0;theta3_=joint_pos3*pi/180.0;
				}
				else solution_exist=false;
				
			}
			else solution_exist=false;
	}
	else solution_exist=false;
	return solution_exist;
}

bool FLX_kinematics::Check_valid(const Matrix4d &Mat_SE3,double &theta1_,double &theta2_,double &theta3_,double &theta4_)
{
	Inverse_Kinematics(Mat_SE3(0,0),Mat_SE3(0,1),Mat_SE3(0,2),Mat_SE3(0,3),
			       Mat_SE3(1,0),Mat_SE3(1,1),Mat_SE3(1,2),Mat_SE3(1,3),
			       Mat_SE3(2,0),Mat_SE3(2,1),Mat_SE3(2,2),Mat_SE3(2,3));
	bool solution_exist=true;
	if (theta.size())
	{
			int valid_n=Valid_Space();
			if (valid_n>0)
			{
				//Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
				double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
				joint_pos1=theta[valid_n-1].theta1*180.0/pi;joint_pos2=theta[valid_n-1].theta2*180.0/pi;joint_pos3=theta[valid_n-1].theta3*180.0/pi;joint_pos4=theta[valid_n-1].theta4*180.0/pi;
				while(joint_pos1>180)joint_pos1-=180;while(joint_pos2>180)joint_pos2-=180;while(joint_pos3>180)joint_pos3-=180;while(joint_pos4>180)joint_pos4-=180;
				if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
				{
					theta1_=joint_pos1*pi/180.0;theta2_=joint_pos2*pi/180.0;theta3_=joint_pos3*pi/180.0;theta4_=joint_pos4*pi/180.0;
				}
				else solution_exist=false;
				
			}
			else solution_exist=false;
	}
	else solution_exist=false;
	return solution_exist;
}

void FLX_kinematics::convert_SE3_tp(const Matrix4d &Mat_SE3,double &theta_e_,double &theta4_,double &px_,double & py_)
{
	theta_e_=atan2(Mat_SE3(0,2),-Mat_SE3(1,2))-pi/2.0;
	theta4_=atan2(Mat_SE3(2,0),Mat_SE3(2,1));
	px_=Mat_SE3(0,3);
	py_=Mat_SE3(1,3);
}