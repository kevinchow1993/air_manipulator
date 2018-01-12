#include <cmath>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

#define  pi 3.14159265358f


class FLX_kinematics
{	
	
	public:
		struct Theta{double theta1,theta2,theta3,theta4;};
		double r11,r12,r13;
		double r21,r22,r23;
		double r31,r32,r33;
		double t1,t2,t3;
		double nx,ox,ax,px;
		double ny,oy,ay,py;
		double nz,oz,az,pz;
		vector<Theta> theta;

		FLX_kinematics();
		~FLX_kinematics();
		void Forward_Kinematics(double angle1,double angle2,double angle3,double angle4);
		void Forward_Link_Kinematics(int links,double angle1=0,double angle2=0,double angle3=0,double angle4=0);
		void Forward_Kinematics(double angle1,double angle2,double angle3,double angle4,Matrix4d &result_SE3);
		void Show_Forward_Result_RT(void);
		void Inverse_Kinematics(double nx0,double ox0,double ax0,double px0,
								double ny0,double oy0,double ay0,double py0,
								double nz0,double oz0,double az0,double pz0);
		void Inverse_Kinematics(double theta_e,double theta4,double px,double py);
		void Show_Inverse_Result_Theta(Theta theta_result);
		double Form_postive_negative_180(double ang);
		void Form_result(Theta &theta_result);

		bool Check_valid(double theta_e,double theta4,double px,double py);
		bool Check_valid(double theta_e,double theta4,double px,double py,double &theta1_,double &theta2_,double &theta3_);
		bool Check_valid(const Matrix4d &Mat_SE3,double &theta1_,double &theta2_,double &theta3_,double &theta4_);
		static bool  Valid_Angle(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4);//360
		int Valid_Space();
		void convert_SE3_tp(const Matrix4d &Mat_SE3,double &theta_e_,double &theta4_,double &px_,double & pz_);



};
