
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/CameraPos.h>
#include <camera_control/TagLost.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 

#include "am_controller/servoset_srv.h"
#include <geometry_msgs/PoseStamped.h>

#include <fstream>
#include <iostream>

#define DATA_RECORD
using namespace std;

//#include "serial_am/ikMsg.h"
// #include "am_controller/servoset_srv.h"

// Create serial port
serial serial;
float last_pose=52.0+48.0;
float Dpose=45.0;
double tag_center_x,tag_center_y;
camera_control::TagLost TagLost_msg;
am_controller::servoset_srv servoset_srv_msg;


int tag_watch_dog=0;


using namespace std;
using namespace Eigen; 
#define pi  3.1415926535898



// double secs;
double recent_marker_time,recent_target_time;
int current_interest_id;
Matrix4d Twb;
Matrix4d Tlc;
Matrix4d Tbl;
Matrix4d Tmb;

bool is_valid_track_target;

int great_70_cnt;


#ifdef DATA_RECORD
	ofstream state_file_recorder;
	double rec_x;
	double rec_y;
	double rec_z;
#endif

void Set_Servo_Pos(int pos)
{
	if (pos>150)pos=150;if (pos<43)pos=43;
	char cmd[100];
	sprintf(cmd,"%ds",pos);
	//printf("%ds\n",pos);
	serial.Write(cmd,strlen(cmd));
	//Dpose=-90.0/96.0*pos+90.0+52.0*90.0/96.0;//k=-0.9375   b=138.75
	Dpose=-0.967742*pos+145.16129;

	if(Dpose>70)great_70_cnt++;
	else great_70_cnt=0;
	if(great_70_cnt>100&&current_interest_id==35)
	{
		tag_watch_dog=3000;
		if(great_70_cnt>4000)great_70_cnt=4000;
	}
}







void set_Tbl_by_Dpose()
{
	double cosb=cos(Dpose/180.0*pi);
	double sinb=sin(Dpose/180.0*pi);
	Tbl(0,0)=cosb;		Tbl(0,1)=0.0;		Tbl(0,2)=sinb;		Tbl(0,3)=0.1218;	//从body到link的偏移量
	Tbl(1,0)=0.0;		Tbl(1,1)=1.0;		Tbl(1,2)=0.0;		Tbl(1,3)=0.0;
	Tbl(2,0)=-sinb;		Tbl(2,1)=0.0;		Tbl(2,2)=cosb;		Tbl(2,3)=-0.1105;
	Tbl(3,0)=0.0;		Tbl(3,1)=0.0;		Tbl(3,2)=0.0;		Tbl(3,3)=1.0;
}

void limited_grasp_call(ros::ServiceClient &servoseter,am_controller::servoset_srv &msg)
{
	static double last_call_time=0;

	if(ros::Time::now().toSec()-last_call_time>0.25)
	{
		servoseter.call(msg);	
		last_call_time=ros::Time::now().toSec();
	}
	
}

void target_detector( geometry_msgs::PoseStamped &pose)
{
	
	    		Quaterniond q(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
			Matrix3d Rca = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
			Vector3d Oca(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
			Matrix4d Tca=MatrixXd::Zero(4,4);Tca(3,3)=1.0;
			Tca<<Rca;Tca.col(3)<<Oca;

			set_Tbl_by_Dpose();

			double target_x,target_y,target_z;
			Matrix4d Tba,Tma;
			Tba=Tbl*Tlc*Tca;
			Tma=Tmb*Tba;
			target_x=Tma(0,3);target_y=Tma(1,3);target_z=Tma(2,3);
			
	

			servoset_srv_msg.request.cmd=6;
			servoset_srv_msg.request.pos1=0;
			servoset_srv_msg.request.pos2=0;
			servoset_srv_msg.request.pos3=target_x;
			servoset_srv_msg.request.pos4=target_y;
			servoset_srv_msg.request.action_time=600;
			//printf("xz=%lf,%lf\n", servoset_srv_msg.request.pos3,servoset_srv_msg.request.pos4);

				// servoseter.call(msg);
				//6 0 0 0.35 -0.09 1000
			#ifdef DATA_RECORD
				state_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<<rec_x<<" "<<rec_y<<" "<<rec_z<<" "<<Tba(0,3)<<" "<<Tba(1,3)<<" "<<Tba(2,3)<<endl;
				printf("%lf,%lf,%lf\n%lf,%lf,%lf\n", rec_x,rec_y,rec_z,Tba(0,3),Tba(1,3),Tba(2,3));
			#endif
}

void CameraPos_CallBack(camera_control::CameraPos msg)
{
	if(msg.id==25)
	{
		recent_target_time =ros::Time::now().toSec(); 
		static int int_valid_25_cnt=0;
		if(int_valid_25_cnt<100)int_valid_25_cnt++;
		if(int_valid_25_cnt>3)current_interest_id=25;
		is_valid_track_target=true;
		target_detector(msg.pose);
		printf("---\nid:%d\nx:%lf \ny:%lf\nz:%lf\n",msg.id,msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
	}

	tag_watch_dog=0;
	TagLost_msg.request.is_lost=0;
	tag_center_x=msg.cx;
	tag_center_y=msg.cy;
	//if (tag_center_y>=250)last_pose--;
	//if (tag_center_y<230)last_pose++;
	float d_err=fabs(tag_center_y-240.0);
	float d_p=0.023,delay_factor=0.0017;//d_p=0.053,delay_factor=0.0017;//


	if (tag_center_y>=250)last_pose-=(d_err*d_p);
	if (tag_center_y<230)last_pose+=(d_err*d_p);

	if (last_pose>52.0+96.0)last_pose=52.0+96.0;if (last_pose<52.0)last_pose=52.0; 
	Set_Servo_Pos(last_pose);
	ros::Duration(delay_factor*d_err*d_p).sleep();

	
	
}

void init_Tlc(void)
{
	//Tlc init
	Tlc=MatrixXd::Zero(4,4);Tlc(3,3)=1.0;
	Quaterniond qlc(-0.484592,0.533285,-0.517225,0.461797);//w x y z but cout<<qld.coffes()  is  x y z w.
	Matrix3d Rlc = qlc.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Olc(0.0394777,-0.00845416,0.0742496);
	Tlc<<Rlc;Tlc.col(3)<<Olc;
}
void init_Tmb(void)
{
	//Tmb init
	Tmb=MatrixXd::Zero(4,4);Tmb(3,3)=1.0;
	Tmb(0,0)=1.0;		Tmb(0,1)=0.0;		Tmb(0,2)=0.0;		Tmb(0,3)=0.04;	//从manipulator到body
	Tmb(1,0)=0.0;		Tmb(1,1)=0.0;		Tmb(1,2)=1.0;		Tmb(1,3)=0.24;
	Tmb(2,0)=0.0;		Tmb(2,1)=-1.0;		Tmb(2,2)=0.0;		Tmb(2,3)=0.0;
	Tmb(3,0)=0.0;		Tmb(3,1)=0.0;		Tmb(3,2)=0.0;		Tmb(3,3)=1.0;
}

void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{
	rec_x=msg->pose.position.x;
  	rec_y=msg->pose.position.y;
  	rec_z=msg->pose.position.z;
 //  	mav_qx=msg->pose.orientation.x;
	// mav_qy=msg->pose.orientation.y;
	// mav_qz=msg->pose.orientation.z;
	// mav_qw=msg->pose.orientation.w;
 //  	mav_t=msg->header.stamp.toSec();
}


int main(int argc, char** argv)
{
	//test_fun();
	init_Tlc();
	init_Tmb();
	ros::init(argc,argv,"target_tracker");
	ros::NodeHandle n;
	std::string com_cs;
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);
	Set_Servo_Pos(last_pose);


	ros::Duration(2.0).sleep();
	ros::Subscriber Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,CameraPos_CallBack);//call by aprilTags pack.
	ros::ServiceClient tag_lost_puber=n.serviceClient<camera_control::TagLost>("/TagLost");

	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");


	/*#ifdef DATA_RECORD
		ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
		state_file_recorder.open("/home/flx/catkin_al/visual_test.txt",ios::out|ios::trunc);
		if(state_file_recorder==NULL)ROS_INFO("NULL FILE PTR");
	#endif
		*/


	ros::Rate loop_rate(1000);
	TagLost_msg.request.is_lost=0;





	// ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	// ros::ServiceClient graper_puber = n.serviceClient<serial_am::graper>("/serial_am/graper");
	//ros::Publisher ik_puber = n.advertise<serial_am::ikMsg>("/Camera_servo_avtion/msg",1);

	// ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
	// secs =ros::Time::now().toSec(); 
	recent_marker_time=0;
	recent_target_time=0;
	current_interest_id=35;

	is_valid_track_target=false;


	while(ros::ok())
	{
		
		if(is_valid_track_target)
			limited_grasp_call(servoseter,servoset_srv_msg);

		tag_watch_dog++;
		if (tag_watch_dog>80)
		{
			is_valid_track_target=false;
		}
		if (tag_watch_dog>2000)
		{
			is_valid_track_target=false;

			if(TagLost_msg.request.is_lost==0)
			{
				TagLost_msg.request.is_lost=1;
				//tag_lost_puber.call(TagLost_msg);
				last_pose=150.0;
			}
			int d_delay=tag_watch_dog-1500;
			if (d_delay%50==0)
			{
				last_pose--;
				Set_Servo_Pos(last_pose);
				if (last_pose==0)last_pose=150.0;
			}

		}



		ros::spinOnce();
		loop_rate.sleep();
	}

	serial.Close();

	#ifdef DATA_RECORD
		state_file_recorder.close();
	#endif

	
		
	
	
	return 0;
}
