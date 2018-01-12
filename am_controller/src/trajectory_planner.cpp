#include "ros/ros.h"
#include <stdio.h>
#include "am_controller/servoset_srv.h"
#include "am_controller/trajectory_paraAction.h"
#include <actionlib/server/simple_action_server.h>

#include "Kin.h"

#include <nsga2/global.h>
#include <nsga2/NSGA2.h>

#include <vector>
#include <iostream>
#include <utility>


#define OUTPUT_TRAJECTORY_FILEDATA



using namespace std;
#define PI 3.14159265358979323846264338327950288419716939937510582097494
/*=========this is global variabl. for known optimal parameter====*/
double theta0_0,theta0_1,theta0_2,thetaf_0,thetaf_1,thetaf_2;
double v0_0,v0_1,v0_2;
double vf_0,vf_1,vf_2;
double global_theta_e,global_theta_4;
double global_max_v,global_max_a;
double obstacle_points_2D_x[5000];
double obstacle_points_2D_y[5000];
int obstacle_points_2D_len=0;
double obstacle_point_gap=0.01;
double obstacle_safe_range=0.01;
double t_gap=0.03;
double t_gap_eva=0.001;
/* ========================================================*/
double buf_x[50000],buf_y[50000];
//final result
am_controller::trajectory_paraResult current_output;
typedef actionlib::SimpleActionServer<am_controller::trajectory_paraAction> Server;

class link_chrom//five order function trajectory generator 
{
	private:
	struct section_bound_condition
	{
		double c[7];//t_f,theta_0,vel_0,acc_0,theta_f,vel_f,acc_f;
	};
	struct outer_bound_contion
	{
		double c[11];//t_a,t_b,theta_0,vel_0,acc_0,theta_m,vel_m,acc_m,theta_f,vel_f,acc_f;
		bool operator == (const outer_bound_contion &a)
		{
			for(int i=0;i<11;i++)if(a.c[i]!=c[i])return false;
			return true;
		}
	};
	struct para_
	{
		double a[6],b[6];
		section_bound_condition con_a,con_b;
		outer_bound_contion con;
	};
	public:
	typedef section_bound_condition bcd;
	typedef outer_bound_contion bc;
	link_chrom(){};
	~link_chrom(){};
	para_ link[3];
	double chrom[29];//t_a,t_b,       ,theta_0,vel_0,acc_0,theta_m,vel_m,acc_m,theta_f,vel_f,acc_f   x3 ... for link 0~2
	void set(
		  double t_a,double t_b,
		  double a0_0,double a0_1,double a0_2,
		  double thetam_0,double thetam_1,double thetam_2,
		  double vm_0,double vm_1,double vm_2,
		  double am_0,double am_1,double am_2,
		  double af_0,double af_1,double af_2
		)//t_a,t_b,       ,theta_0,vel_0,acc_0,theta_m,vel_m,acc_m,theta_f,vel_f,acc_f   x3 ... for link 0~2
	{

		//t_a,t_b,theta_0,vel_0,acc_0,theta_m,vel_m,acc_m,theta_f,vel_f,acc_f;
		chrom[0]=t_a;chrom[1]=t_b;

		chrom[2]=theta0_0;chrom[3]=v0_0;chrom[4]=a0_0;
		chrom[5]=thetam_0;chrom[6]=vm_0;chrom[7]=am_0;
		chrom[8]=thetaf_0;chrom[9]=vf_0;chrom[10]=af_0;

		chrom[11]=theta0_1;chrom[12]=v0_1;chrom[13]=a0_1;
		chrom[14]=thetam_1;chrom[15]=vm_1;chrom[16]=am_1;
		chrom[17]=thetaf_1;chrom[18]=vf_1;chrom[19]=af_1;

		chrom[20]=theta0_2;chrom[21]=v0_2;chrom[22]=a0_2;
		chrom[23]=thetam_2;chrom[24]=vm_2;chrom[25]=am_2;
		chrom[26]=thetaf_2;chrom[27]=vf_2;chrom[28]=af_2;
		



	}
	void set_para_condition(double _p[],bcd &condition)
	{
		_p[0]=condition.c[1];
		_p[1]=condition.c[2];
		_p[2]=condition.c[3]/2.0;
		_p[3]=	(
				20.0*condition.c[4]-20.0*condition.c[1]
				-(8.0*condition.c[5]+12.0*condition.c[2])*condition.c[0]
				-(3.0*condition.c[3]-condition.c[6])*condition.c[0]*condition.c[0]
			)/(2.0*condition.c[0]*condition.c[0]*condition.c[0]);
		_p[4]=	(
				-30.0*condition.c[4]+30.0*condition.c[1]
				+(14.0*condition.c[5]+16.0*condition.c[2])*condition.c[0]
				+(3.0*condition.c[3]-2.0*condition.c[6])*condition.c[0]*condition.c[0]
			)/(2.0*condition.c[0]*condition.c[0]*condition.c[0]*condition.c[0]);
		_p[5]=	(
				12.0*condition.c[4]-12.0*condition.c[1]
				-(6.0*condition.c[5]+6.0*condition.c[2])*condition.c[0]
				-(condition.c[3]-condition.c[6])*condition.c[0]*condition.c[0]
			)/(2.0*condition.c[0]*condition.c[0]*condition.c[0]*condition.c[0]*condition.c[0]);
	}
	void calculate_link_para(int link_no,bcd condition_a,bcd condition_b)
	{
		for(int i=0;i<7;i++)link[link_no].con_a.c[i]=condition_a.c[i];
		for(int i=0;i<7;i++)link[link_no].con_b.c[i]=condition_b.c[i];
		set_para_condition(link[link_no].a,link[link_no].con_a);
		set_para_condition(link[link_no].b,link[link_no].con_b);
	}
	void set_link_para(int link_no,bc condition)
	{
		bcd ca={condition.c[0],condition.c[2],condition.c[3],condition.c[4],condition.c[5],condition.c[6],condition.c[7]};
		bcd cb={condition.c[1],condition.c[5],condition.c[6],condition.c[7],condition.c[8],condition.c[9],condition.c[10]};
		calculate_link_para(link_no,ca,cb);
	}
	void calculate_param_ab()
	{
		for(int i=0;i<3;i++)
		{
			bc cc={chrom[0],chrom[1],chrom[2+i*9],chrom[3+i*9],chrom[4+i*9],chrom[5+i*9],chrom[6+i*9],chrom[7+i*9],chrom[8+i*9],chrom[9+i*9],chrom[10+i*9]};
			set_link_para(i,cc);
		}
	}
	double get_link_angle_a(int link_no,double t)
	{
		return link[link_no].a[0]+link[link_no].a[1]*t+link[link_no].a[2]*t*t+link[link_no].a[3]*t*t*t+link[link_no].a[4]*t*t*t*t+link[link_no].a[5]*t*t*t*t*t;
	}
	double get_link_angle_b(int link_no,double t)
	{
		return link[link_no].b[0]+link[link_no].b[1]*t+link[link_no].b[2]*t*t+link[link_no].b[3]*t*t*t+link[link_no].b[4]*t*t*t*t+link[link_no].b[5]*t*t*t*t*t;
	}
	double get_link_vel_a(int link_no,double t)
	{
		return link[link_no].a[1]+2*link[link_no].a[2]*t+3*link[link_no].a[3]*t*t+4*link[link_no].a[4]*t*t*t+5*link[link_no].a[5]*t*t*t*t;
	}
	double get_link_vel_b(int link_no,double t)
	{
		return link[link_no].b[1]+2*link[link_no].b[2]*t+3*link[link_no].b[3]*t*t+4*link[link_no].b[4]*t*t*t+5*link[link_no].b[5]*t*t*t*t;
	}
	double get_link_acc_a(int link_no,double t)
	{
		return 2*link[link_no].a[2]+6*link[link_no].a[3]*t+12*link[link_no].a[4]*t*t+20*link[link_no].a[5]*t*t*t;
	}
	double get_link_acc_b(int link_no,double t)
	{
		return 2*link[link_no].b[2]+6*link[link_no].b[3]*t+12*link[link_no].b[4]*t*t+20*link[link_no].b[5]*t*t*t;
	}
	void get_output(vector<double> &result)
	{
		result.clear();
		for(int i=0;i<3;i++)
			for(int j=0;j<6;j++)
			{
				result.push_back(link[i].a[j]);
				result.push_back(link[i].b[j]);
				// cout<<link[i].a[j]<<endl<<link[i].b[j]<<endl;
			}

	}


};

/*
	已知：
	1末端执行器起始位置和目标位置
		得到每个关节的起始位置和目标位置。（弧度）时间：（秒）
	约束：
	1末端执行器起始位置和目标位置，得到每个关节的起始位置和目标位置。（弧度）时间：（秒）
	 每个关节的起始速度和终止速度为0（或设定值），速度不超过+-maxv，加速度不超过 +-maxa，时间t_a、t_b都在[0,t_set]
	2关节空间约束：每个关节角度约束
		正逆解约束、舵机行程约束
	3轨迹空间障碍物约束

	优化目标：
	1时间最短：t_a+t_b最小
	2轨迹最短：对时间进行离散化，对关节函数每10ms采一次每个关节位置，正运动学解得到关节位置，相邻点距离求和
	3---
*/
bool Valid_vel(double vel_joint1,double vel_joint2,double vel_joint3)
{
	if (vel_joint1<-global_max_v||vel_joint2<-global_max_v||vel_joint3<-global_max_v||vel_joint1>global_max_v||vel_joint2>global_max_v||vel_joint3>global_max_v)return false;
	else return true;
}
bool Valid_acc(double acc_joint1,double acc_joint2,double acc_joint3)
{
	if (acc_joint1<-global_max_a||acc_joint2<-global_max_a||acc_joint3<-global_max_a||acc_joint1>global_max_a||acc_joint2>global_max_a||acc_joint3>global_max_a)return false;
	else return true;
}
bool  Valid_Angle(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4)//360
{
		// double servo_pos1=5.0-joint_pos1;
		// double servo_pos2=1.2*joint_pos2-25.0;
		// double servo_pos3=-1.1*joint_pos2 -0.64444*joint_pos3+207;
		// double servo_pos4=joint_pos4+2;
		// if (servo_pos1<0||servo_pos2<0||servo_pos3<0||servo_pos4<0||servo_pos1>180||servo_pos2>180||servo_pos3>180||servo_pos4>180)
		// {
		// 	return false;
		// }
		// else return true;
	return FLX_kinematics::Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4);
}

double distance(double x,double y,double ox,double oy) 
{
    return sqrt((x-ox)*(x-ox)+(y-oy)*(y-oy));
}
double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)  
{  
	    double ans = 0;
	    double a, b, c;
	    a = distance(x1,y1,x2,y2);
	    b = distance(x1,y1,x,y);
	    c = distance(x2,y2,x,y);
	    if (c+b==a) {//点在线段上
	      ans = 0;
	      return ans;
	    }
	    if (a<=0.00001) {//不是线段，是一个点
	      ans = b;
	      return ans;
	    }
	    if (c*c >= a*a + b*b) { //组成直角三角形或钝角三角形，p1为直角或钝角
	      ans = b;
	      return ans;
	    }
	    if (b * b >= a * a + c * c) {// 组成直角三角形或钝角三角形，p2为直角或钝角
	      ans = c;
	      return ans;
	    }
	    // 组成锐角三角形，则求三角形的高
	    double p0 = (a + b + c) / 2;// 半周长
	    double s = sqrt(p0 * (p0 - a) * (p0 - b) * (p0 - c));// 海伦公式求面积
	    ans = 2*s / a;// 返回点到线的距离（利用三角形面积公式求高）
	    return ans;


}  
double obstacle_hit_range(double x,double y)
{
	double hit_cnt=0.0;
	double ox,oy,obs_gap;
	for(int i=0;i<obstacle_points_2D_len;i++)
	{
		ox=obstacle_points_2D_x[i];
		oy=obstacle_points_2D_y[i];
		obs_gap=sqrt((x-ox)*(x-ox)+(y-oy)*(y-oy));
		if(obs_gap<=obstacle_safe_range)hit_cnt+=1;//((obstacle_safe_range-obs_gap)*(obstacle_safe_range-obs_gap));
	}
	return hit_cnt;
}

double obstacle_hit_range_links(double x[],double y[])
{
	double hit_cnt=0.0;
	double ox,oy,obs_gap;
	for(int i=0;i<obstacle_points_2D_len;i++)
	{
		ox=obstacle_points_2D_x[i];
		oy=obstacle_points_2D_y[i];
		for(int j=0;j<3;j++)
		{
			obs_gap=PointToSegDist(ox,oy,x[j],y[j],x[j+1],y[j+1]);
			if(obs_gap<=obstacle_safe_range)hit_cnt+=1;//((obstacle_safe_range-obs_gap)*(obstacle_safe_range-obs_gap));
		}
	}
	return hit_cnt;

}
bool get_Kinematic_valid(double theta_e,double theta4,double px,double py,double &theta1,double &theta2,double &theta3)
{
		FLX_kinematics Kin; 
		return Kin.Check_valid(theta_e,theta4,px,py,theta1,theta2,theta3);

}
void trajectory_evaluate_model(double *xreal,
	                    double *xbin,
	                    int **gene,
	                    double *obj,
	                    double *constr) 
{
	 FLX_kinematics Kin; 
	//decode from real variable.
	    double t_a = xreal[0];		double t_b = xreal[1];
	    double a0_0 = xreal[2];	double a0_1 = xreal[3];		double a0_2 = xreal[4];
	    double thetam_0 = xreal[5];	double thetam_1 = xreal[6];	double thetam_2 = xreal[7];
	    double vm_0 = xreal[8];	double vm_1 = xreal[9];		double vm_2 = xreal[10];
	    double am_0 = xreal[11];	double am_1 = xreal[12];	double am_2 = xreal[13];
	    double af_0 = xreal[14];	double af_1 = xreal[15];		double af_2 = xreal[16];

	 //calculate links ab
	    link_chrom links_opt;
	    links_opt.set(t_a,t_b,a0_0,a0_1,a0_2,thetam_0,thetam_1,thetam_2,vm_0,vm_1,vm_2,am_0,am_1,am_2,af_0,af_1,af_2);
	    links_opt.calculate_param_ab();
	    obj[0] =t_a+t_b;//optimal objective1
	    int buf_length;
	    buf_length=0;
	    constr[0]=0;
	    constr[1]=0;
	    constr[2]=0;
	    double t=0.0;
	    double ang_joint1,ang_joint2,ang_joint3,vel_joint1,vel_joint2,vel_joint3,acc_joint1,acc_joint2,acc_joint3;
	    double kin_x[4],kin_y[4];
	    // obj[2]=0;
	    // double abs_acc1;
	    for(;t<=t_a;t+=t_gap)
	    {
	    	ang_joint1=links_opt.get_link_angle_a(0,t);ang_joint2=links_opt.get_link_angle_a(1,t);ang_joint3=links_opt.get_link_angle_a(2,t);
	    	vel_joint1=links_opt.get_link_vel_a(0,t);vel_joint2=links_opt.get_link_vel_a(1,t);vel_joint3=links_opt.get_link_vel_a(2,t);
	    	acc_joint1=links_opt.get_link_acc_a(0,t);acc_joint2=links_opt.get_link_acc_a(1,t);acc_joint3=links_opt.get_link_acc_a(2,t);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr[0]-=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr[0]-=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr[0]-=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr[0]-=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr[0]-=1;
	    	buf_x[buf_length]=Kin.t1;buf_y[buf_length]=Kin.t2;
	    	buf_length++;
	    	// constr[3]-=obstacle_hit_range(Kin.t1,Kin.t2);

	    	// abs_acc1=fabs(acc_joint1);
	    	// if(obj[2]<abs_acc1)obj[2]=abs_acc1;
	    }
	    for(;t<=t_a+t_b;t+=t_gap)
	    {
	    	ang_joint1=links_opt.get_link_angle_b(0,t-t_a);ang_joint2=links_opt.get_link_angle_b(1,t-t_a);ang_joint3=links_opt.get_link_angle_b(2,t-t_a);
	    	vel_joint1=links_opt.get_link_vel_b(0,t-t_a);vel_joint2=links_opt.get_link_vel_b(1,t-t_a);vel_joint3=links_opt.get_link_vel_b(2,t-t_a);
	    	acc_joint1=links_opt.get_link_acc_b(0,t-t_a);acc_joint2=links_opt.get_link_acc_b(1,t-t_a);acc_joint3=links_opt.get_link_acc_b(2,t-t_a);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr[0]-=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr[0]-=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr[0]-=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr[0]-=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr[0]-=1;
	    	buf_x[buf_length]=Kin.t1;buf_y[buf_length]=Kin.t2;
	    	buf_length++;
	    	// constr[3]-=obstacle_hit_range(Kin.t1,Kin.t2);

	    	// abs_acc1=fabs(acc_joint1);
	    	// if(obj[2]<abs_acc1)obj[2]=abs_acc1;
	    }
	    if(1)
	    {
		t=t_a+t_b;
		ang_joint1=links_opt.get_link_angle_b(0,t-t_a);ang_joint2=links_opt.get_link_angle_b(1,t-t_a);ang_joint3=links_opt.get_link_angle_b(2,t-t_a);
	    	vel_joint1=links_opt.get_link_vel_b(0,t-t_a);vel_joint2=links_opt.get_link_vel_b(1,t-t_a);vel_joint3=links_opt.get_link_vel_b(2,t-t_a);
	    	acc_joint1=links_opt.get_link_acc_b(0,t-t_a);acc_joint2=links_opt.get_link_acc_b(1,t-t_a);acc_joint3=links_opt.get_link_acc_b(2,t-t_a);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr[0]-=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr[0]-=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr[0]-=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr[0]-=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr[0]-=1;
	    	buf_x[buf_length]=Kin.t1;buf_y[buf_length]=Kin.t2;
	    	buf_length++;
	    }
	

	    obj[1]=0; //optimal object1
	    double x_gap,y_gap;
	    for (int i = 0; i < buf_length-1; ++i)
	    {
	    	x_gap=buf_x[i+1]-buf_x[i];
	    	y_gap=buf_y[i+1]-buf_y[i];
	    	obj[1]+=sqrt(x_gap*x_gap+y_gap*y_gap);
	    	// obj[1]+=(x_gap*x_gap+y_gap*y_gap);
	    }
	    

	    // obj[2]+= 0.2*obj[1]+obj[0];
	   
	    return;
}

bool result_evaluate(	double t_a,double t_b,
			double a0_0,double a0_1,double a0_2,
			double thetam_0,double thetam_1,double thetam_2,
			double vm_0,double vm_1,double vm_2,
			double am_0,double am_1,double am_2,
			double af_0,double af_1,double af_2)
{

	#ifdef OUTPUT_TRAJECTORY_FILEDATA
		std::ofstream plot_data;
		plot_data.open("plot_data.out" , ios::out | ios::trunc);
	#endif


	   link_chrom links_opt;
	    links_opt.set(t_a,t_b,a0_0,a0_1,a0_2,thetam_0,thetam_1,thetam_2,vm_0,vm_1,vm_2,am_0,am_1,am_2,af_0,af_1,af_2);
	    links_opt.calculate_param_ab();
	    FLX_kinematics Kin; 
	    int buf_length;
	    buf_length=0;
	    bool constr=0;
	    double t=0.0;
	    double ang_joint1,ang_joint2,ang_joint3,vel_joint1,vel_joint2,vel_joint3,acc_joint1,acc_joint2,acc_joint3;
	    double kin_x[4],kin_y[4];
	    for(;t<=t_a;t+=t_gap_eva)
	    {
	    	ang_joint1=links_opt.get_link_angle_a(0,t);ang_joint2=links_opt.get_link_angle_a(1,t);ang_joint3=links_opt.get_link_angle_a(2,t);
	    	vel_joint1=links_opt.get_link_vel_a(0,t);vel_joint2=links_opt.get_link_vel_a(1,t);vel_joint3=links_opt.get_link_vel_a(2,t);
	    	acc_joint1=links_opt.get_link_acc_a(0,t);acc_joint2=links_opt.get_link_acc_a(1,t);acc_joint3=links_opt.get_link_acc_a(2,t);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr=1;
	    	#ifdef OUTPUT_TRAJECTORY_FILEDATA
		    	plot_data<<ang_joint1<<" "<<ang_joint2<<" "<<ang_joint3<<" "<<vel_joint1<<" "<<vel_joint2<<" "<<vel_joint3<<" "<<acc_joint1<<" "<<acc_joint2<<" "<<acc_joint3<<" ";
		    	plot_data<<kin_x[0]<<" "<<kin_y[0]<<" "<<kin_x[1]<<" "<<kin_y[1]<<" "<<kin_x[2]<<" "<<kin_y[2]<<" "<<kin_x[3]<<" "<<kin_y[3]<<" "<<constr<<" "<<t<<endl;
	    	#endif 
	    }
	    for(;t<=t_a+t_b;t+=t_gap_eva)
	    {
	    	ang_joint1=links_opt.get_link_angle_b(0,t-t_a);ang_joint2=links_opt.get_link_angle_b(1,t-t_a);ang_joint3=links_opt.get_link_angle_b(2,t-t_a);
	    	vel_joint1=links_opt.get_link_vel_b(0,t-t_a);vel_joint2=links_opt.get_link_vel_b(1,t-t_a);vel_joint3=links_opt.get_link_vel_b(2,t-t_a);
	    	acc_joint1=links_opt.get_link_acc_b(0,t-t_a);acc_joint2=links_opt.get_link_acc_b(1,t-t_a);acc_joint3=links_opt.get_link_acc_b(2,t-t_a);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr=1;
	    	#ifdef OUTPUT_TRAJECTORY_FILEDATA
		    	plot_data<<ang_joint1<<" "<<ang_joint2<<" "<<ang_joint3<<" "<<vel_joint1<<" "<<vel_joint2<<" "<<vel_joint3<<" "<<acc_joint1<<" "<<acc_joint2<<" "<<acc_joint3<<" ";
		    	plot_data<<kin_x[0]<<" "<<kin_y[0]<<" "<<kin_x[1]<<" "<<kin_y[1]<<" "<<kin_x[2]<<" "<<kin_y[2]<<" "<<kin_x[3]<<" "<<kin_y[3]<<" "<<constr<<" "<<t<<endl;
	    	#endif
	    }
	     if(1)
	    {
		t=t_a+t_b;
		ang_joint1=links_opt.get_link_angle_b(0,t-t_a);ang_joint2=links_opt.get_link_angle_b(1,t-t_a);ang_joint3=links_opt.get_link_angle_b(2,t-t_a);
	    	vel_joint1=links_opt.get_link_vel_b(0,t-t_a);vel_joint2=links_opt.get_link_vel_b(1,t-t_a);vel_joint3=links_opt.get_link_vel_b(2,t-t_a);
	    	acc_joint1=links_opt.get_link_acc_b(0,t-t_a);acc_joint2=links_opt.get_link_acc_b(1,t-t_a);acc_joint3=links_opt.get_link_acc_b(2,t-t_a);
	    	if(Valid_Angle(ang_joint1*180.0/pi,ang_joint2*180.0/pi,ang_joint3*180.0/pi,global_theta_4*180.0/pi)==false)constr=1;
	    	if(Valid_vel(vel_joint1, vel_joint2,vel_joint3)==false)constr=1;
	    	if(Valid_acc(acc_joint1, acc_joint2,acc_joint3)==false)constr=1;
	    	for(int i=1;i<=4;i++)
	    	{
	    		Kin.Forward_Link_Kinematics(i,ang_joint1,ang_joint2,ang_joint3,global_theta_4);
	    		kin_x[i-1]=Kin.t1;kin_y[i-1]=Kin.t2;
	    		// if(obstacle_hit_range(Kin.t1,Kin.t2)>0.0001)constr=1;
	    	}
	    	if(obstacle_hit_range_links(kin_x,kin_y)>0.0001)constr=1;
	    	#ifdef OUTPUT_TRAJECTORY_FILEDATA
		    	plot_data<<ang_joint1<<" "<<ang_joint2<<" "<<ang_joint3<<" "<<vel_joint1<<" "<<vel_joint2<<" "<<vel_joint3<<" "<<acc_joint1<<" "<<acc_joint2<<" "<<acc_joint3<<" ";
		    	plot_data<<kin_x[0]<<" "<<kin_y[0]<<" "<<kin_x[1]<<" "<<kin_y[1]<<" "<<kin_x[2]<<" "<<kin_y[2]<<" "<<kin_x[3]<<" "<<kin_y[3]<<" "<<constr<<" "<<t<<endl;
	    	#endif
	    }

	    #ifdef OUTPUT_TRAJECTORY_FILEDATA
	 	   plot_data.close();
	    #endif
	    if(!constr)
	    {
	    	links_opt.get_output(current_output.para);
	    	current_output.t_a=t_a;
	    	current_output.t_b=t_b;
	    }
	    return !constr;
}

bool NSGA2_Trajectory_optmal(nsga2::individual_config &conf)
{

		 nsga2::NSGA2 nsga2_opt;
		 //there are total 17 real variable 
		int seed = time(0);
   		// cout << "Using seed " << seed << endl;
		nsga2_opt.set_seed(seed);
		nsga2_opt.set_nreal(17);
		nsga2_opt.set_nbin(0);
		nsga2_opt.set_nobj(2);
		nsga2_opt.set_ncon(1); // add a constraint due to possible simulation failures
		nsga2_opt.set_popsize(60);
		nsga2_opt.set_ngen(150);
		nsga2_opt.set_pcross_real(1.0);
		nsga2_opt.set_pcross_bin(0.0);
		nsga2_opt.set_pmut_real(0.5);
		nsga2_opt.set_pmut_bin(0.0);
		nsga2_opt.set_eta_c(50);
		nsga2_opt.set_eta_m(20);
		nsga2_opt.set_epsilon_c(1e-14);
		nsga2_opt.set_limits_realvar(conf.limits_realvar);
		nsga2_opt.set_function(&trajectory_evaluate_model);
		nsga2_opt.set_crowdobj(false); // crowd over the parameters, not the objective functions
		nsga2_opt.set_backup_filename(""); // no backup


		nsga2_opt.initialize();
		nsga2_opt.evolve();
		
		std::vector<nsga2::individual>::const_iterator it_pop;
		it_pop=nsga2_opt.parent_pop->ind.begin();
		if(result_evaluate(it_pop->xreal[0],it_pop->xreal[1],
				it_pop->xreal[2],it_pop->xreal[3],it_pop->xreal[4],
				it_pop->xreal[5],it_pop->xreal[6],it_pop->xreal[7],
				it_pop->xreal[8],it_pop->xreal[9],it_pop->xreal[10],
				it_pop->xreal[11],it_pop->xreal[12],it_pop->xreal[13],
				it_pop->xreal[14],it_pop->xreal[15],it_pop->xreal[16]
			))
		{
			
			cout<<"obj1:"<<it_pop->obj[0]<<endl;
			cout<<"obj2:"<<it_pop->obj[1]<<endl;
			// cout<<"obj3:"<<it_pop->obj[2]<<endl;
			cout<<"t_a:"<<it_pop->xreal[0]<<endl;
			cout<<"t_b:"<<it_pop->xreal[1]<<endl;
			cout<<"a0_0:"<<it_pop->xreal[2]<<endl;
			cout<<"a0_1:"<<it_pop->xreal[3]<<endl;
			cout<<"a0_2:"<<it_pop->xreal[4]<<endl;
			cout<<"thetam_0:"<<it_pop->xreal[5]<<endl;
			cout<<"thetam_1:"<<it_pop->xreal[6]<<endl;
			cout<<"thetam_2:"<<it_pop->xreal[7]<<endl;
			cout<<"vm_0:"<<it_pop->xreal[8]<<endl;
			cout<<"vm_1:"<<it_pop->xreal[9]<<endl;
			cout<<"vm_2:"<<it_pop->xreal[10]<<endl;
			cout<<"am_0:"<<it_pop->xreal[11]<<endl;
			cout<<"am_1:"<<it_pop->xreal[12]<<endl;
			cout<<"am_2:"<<it_pop->xreal[13]<<endl;
			cout<<"af_0:"<<it_pop->xreal[14]<<endl;
			cout<<"af_1:"<<it_pop->xreal[15]<<endl;
			cout<<"af_2:"<<it_pop->xreal[16]<<endl;
			return 1;
		}
		else
		{
			printf("Solution infeasible...\n");
			return 0;
		}

		
}
void obstacle_generator(double dx,double dy)
{
	std::ofstream plot_data;
	plot_data.open("obstacle_2D_data.out" , ios::out | ios::trunc);
	obstacle_points_2D_len=0;
	for(double ox=dx-0.11;ox<=dx+0.11;ox+=obstacle_point_gap)//for(double ox=dx-0.03;ox<=dx+0.03;ox+=obstacle_point_gap)
	{
		double oz=dy+0.02;
		obstacle_points_2D_x[obstacle_points_2D_len]=ox;
		obstacle_points_2D_y[obstacle_points_2D_len]=oz;
		obstacle_points_2D_len++;
		plot_data<<ox<<" "<<oz<<endl;
	}
	for(double ox=dx-0.04;ox<=dx+0.04;ox+=obstacle_point_gap)
	{
		double oz=dy-0.07;
		obstacle_points_2D_x[obstacle_points_2D_len]=ox;
		obstacle_points_2D_y[obstacle_points_2D_len]=oz;
		obstacle_points_2D_len++;
		plot_data<<ox<<" "<<oz<<endl;
	}
	for(double oz=dy+0.02;oz>=dy-0.07;oz-=obstacle_point_gap)
	{
		double ox=dx+obstacle_safe_range+0.000001;
		obstacle_points_2D_x[obstacle_points_2D_len]=ox;
		obstacle_points_2D_y[obstacle_points_2D_len]=oz;
		obstacle_points_2D_len++;
		plot_data<<ox<<" "<<oz<<endl;
	}

	double ox=dx-0.18,oz=dy-0.09;
	obstacle_points_2D_x[obstacle_points_2D_len]=ox;
	obstacle_points_2D_y[obstacle_points_2D_len]=oz;
	obstacle_points_2D_len++;
	plot_data<<ox<<" "<<oz<<endl;

	plot_data.close();
}
bool make_trajectory_NSGA2(double theta_e,double theta4,double cx,double cy,double dx,double dy,double v0_0_,double v0_1_,double v0_2_,double maxv,double maxa,double maxt)
{
		obstacle_generator(dx,dy);
		global_theta_e=theta_e;
		global_theta_4=theta4;
		global_max_v=maxv;
		global_max_a=maxa;
		//calculate theta0_0,theta0_1,theta0_2,thetaf_0,thetaf_1,thetaf_2
		if(get_Kinematic_valid(theta_e,theta4,cx,cy,theta0_0,theta0_1,theta0_2)==false){printf("kinematic info: invalid start point.\n");return 0;}
		if(get_Kinematic_valid(theta_e,theta4,dx,dy,thetaf_0,thetaf_1,thetaf_2)==false){printf("kinematic info: invalid target point.\n");return 0;}
		nsga2::individual_config conf;
		conf.limits_realvar.push_back(make_pair(0.2,maxt));//t_a
		conf.limits_realvar.push_back(make_pair(0.2,maxt));//t_b
		//theta0_0 set to known by get_Kinematic_valid
		//theta0_1 set to known by get_Kinematic_valid
		//theta0_2 set to known by get_Kinematic_valid
		v0_0=v0_0_;//v0_0 set to known
		v0_1=v0_1_;//v0_1 set to known
		v0_2=v0_2_;//v0_2 set to known
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//a0_0
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//a0_1
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//a0_2
		conf.limits_realvar.push_back(make_pair(-PI/5.0*4.0,	0.0));//thetam_0
		conf.limits_realvar.push_back(make_pair( PI/90.0,	PI));//thetam_1
		conf.limits_realvar.push_back(make_pair( 0.0,	1.33*PI));//thetam_2
		conf.limits_realvar.push_back(make_pair(-maxv,maxv));//vm_0
		conf.limits_realvar.push_back(make_pair(-maxv,maxv));//vm_1
		conf.limits_realvar.push_back(make_pair(-maxv,maxv));//vm_2
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//am_0
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//am_1
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//am_2
		//thetaf_0 set to known by get_Kinematic_valid
		//thetaf_1 set to known by get_Kinematic_valid
		//thetaf_2 set to known by get_Kinematic_valid
		vf_0=0;//set to zero
		vf_1=0;//set to zero
		vf_2=0;//set to zero
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//af_0
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//af_1
		conf.limits_realvar.push_back(make_pair(-maxa,maxa));//af_2
		
		return NSGA2_Trajectory_optmal(conf);
}





void execute(const am_controller::trajectory_paraGoalConstPtr& goal, Server* as)
{

	double t_start=ros::Time::now().toSec();
	current_output.para.clear();
	//(double theta_e,double theta4,double cx,double cy,double dx,double dy,double v0_0_,double v0_1_,double v0_2_,double maxv,double maxa,double maxt)
	if(make_trajectory_NSGA2(
				goal->theta_e,goal->theta4,//theta_e,theta4,
				goal->cx,goal->cy,//cx cy
				goal->dx,goal->dy,//dx dy
				goal->v0_0,goal->v0_1,goal->v0_2,//v0_0  v0_1 v0_2
				PI,PI,8.0//maxv maxa maxt
				)
	)
	{

		double t_used=ros::Time::now().toSec()-t_start;
		printf("Planning Time used:%.3lfms\n",t_used*1000.0 );

		as->setSucceeded(current_output);
		//system("python ~/catkin_al/python_plot.py");
	}
	else 
	{
		current_output.t_a=0;
		current_output.t_b=0;
		double t_used=ros::Time::now().toSec()-t_start;
		printf("Generate trajectory failed, time used:%.3lfms\n",t_used*1000.0 );
		as->setSucceeded(current_output);
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"trajectory_planner");
	ros::NodeHandle n;
	


	 Server server(n, "/trajectory_nsga2_generator", boost::bind(&execute, _1, &server), false);
	 server.start();
	 ros::spin();



	return 0;

}