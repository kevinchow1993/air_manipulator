
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/CameraPos.h>
#include <camera_control/TagLost.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 


//#include "serial_am/ikMsg.h"
#include "am_controller/servoset_srv.h"

// Create serial port
serial serial;
float last_pose=52.0+48.0;
float Dpose=45.0;
double tag_center_x,tag_center_y;
camera_control::TagLost TagLost_msg;




int tag_watch_dog=0;


using namespace std;
using namespace Eigen; 
#define pi  3.1415926535898





void Set_Servo_Pos(int pos)
{
	if (pos>150)pos=150;if (pos<43)pos=43;
	char cmd[100];
	sprintf(cmd,"%ds",pos);
	//printf("%ds\n",pos);
	serial.Write(cmd,strlen(cmd));
	//Dpose=-90.0/96.0*pos+90.0+52.0*90.0/96.0;//k=-0.9375   b=138.75
	Dpose=-0.967742*pos+145.16129;
}


bool ServoPos_CallBack(camera_control::ServoPos::Request &msg,camera_control::ServoPos::Response &res)
{
	Set_Servo_Pos(msg.servo_pos);
	res.is_done=1;
	return true;
}

void CameraPos_CallBack(camera_control::CameraPos msg)
{
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

void watch_dog_refind_tag(void)
{
	//static int 
}


void test_fun(void)
{
	// Matrix4d T=MatrixXd::Ones(4,4);
	// Matrix3d O=MatrixXd::Zero(3,3);
	// Vector3d V(1,2,3);
	// T<<O;
	// T.col(3)<<V;
	// cout<<T<<endl;
	// Quaterniond qlc(-0.551381,0.564901,-0.405939,0.460521);
	// cout<<qlc.coeffs();

	getchar();
}


int main(int argc, char** argv)
{
	//test_fun();

	ros::init(argc,argv,"camera_control");
	ros::NodeHandle n;
	std::string com_cs;
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);
	Set_Servo_Pos(last_pose);


	ros::Duration(2.0).sleep();
	ros::ServiceServer ServoPos_service = n.advertiseService("/Camera_servo/pos_seter", ServoPos_CallBack);
	ros::Subscriber Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,CameraPos_CallBack);
	ros::ServiceClient tag_lost_puber=n.serviceClient<camera_control::TagLost>("/TagLost");


	tf::TransformListener listener;

	ros::Rate loop_rate(1000);
	TagLost_msg.request.is_lost=0;

	//Tlc init
	Matrix4d Tlc=MatrixXd::Zero(4,4);Tlc(3,3)=1.0;
	Quaterniond qlc(-0.484592,0.533285,-0.517225,0.461797);//w x y z but cout<<qld.coffes()  is  x y z w.
	Matrix3d Rlc = qlc.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Olc(0.0394777,-0.00845416,0.0742496);
	Tlc<<Rlc;Tlc.col(3)<<Olc;



	// ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	// ros::ServiceClient graper_puber = n.serviceClient<serial_am::graper>("/serial_am/graper");
	//ros::Publisher ik_puber = n.advertise<serial_am::ikMsg>("/Camera_servo_avtion/msg",1);

	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
	double secs =ros::Time::now().toSec(); 
	while(ros::ok())
	{
		

		tf::StampedTransform transform;
		bool valid_transform;
		try
		{
	    		valid_transform=true;
			listener.lookupTransform("/camera", "/target1", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
		    	valid_transform=false;
		    	ROS_ERROR("%s",ex.what());
		}
		if(valid_transform)
		{
	    		Quaterniond q(transform.getRotation().w(),transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z());
			Matrix3d Rca = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
			Vector3d Oca(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
			Matrix4d Tca=MatrixXd::Zero(4,4);Tca(3,3)=1.0;
			Tca<<Rca;Tca.col(3)<<Oca;

			Matrix4d Tbl;
			double cosb=cos(Dpose/180.0*pi);
			double sinb=sin(Dpose/180.0*pi);
			Tbl(0,0)=cosb;		Tbl(0,1)=0.0;		Tbl(0,2)=sinb;		Tbl(0,3)=0.1218;	//从body到link的偏移量
			Tbl(1,0)=0.0;		Tbl(1,1)=1.0;		Tbl(1,2)=0.0;		Tbl(1,3)=0.0;
			Tbl(2,0)=-sinb;		Tbl(2,1)=0.0;		Tbl(2,2)=cosb;		Tbl(2,3)=-0.1105;
			Tbl(3,0)=0.0;		Tbl(3,1)=0.0;		Tbl(3,2)=0.0;		Tbl(3,3)=1.0;

			double target_x,target_y,target_z;
			Matrix4d Tba;
			Tba=Tbl*Tlc*Tca;
			target_x=Tba(0,3);target_y=Tba(1,3);target_z=Tba(2,3);
			Matrix3d M_rotation;
			M_rotation<<Tba;
			Quaterniond q_angles(M_rotation);
			double x,y,z,w,rol,pit,yaw;
			x=q_angles.x();y=q_angles.y();z=q_angles.z();w=q_angles.w();
		    	rol=atan2(2*w*x+2*y*z,1-2*x*x-2*y*y);
		    	pit=asin(2*w*y-2*z*x);
		    	yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
	    	
			//cout<<"--------tag_Info-----\n"<<target_x<<"\t\t"<<target_y<<"\t\t"<<target_z<<endl;
			//cout<<rol*180.0/pi<<"\t\t"<<pit*180.0/pi<<"\t\t"<<yaw*180.0/pi<<endl;

			// serial_am::ikMsg msg;
			// msg.theta_e=0;
			// msg.theta4=pi/2;
			// msg.px=target_x+0.06;
			// msg.py=target_z+0.24;
			// ik_puber.publish(msg);

			am_controller::servoset_srv msg;
			msg.request.cmd=6;
			msg.request.pos1=0;
			msg.request.pos2=0;//pi/2;
			msg.request.pos3=target_x+0.04;
			msg.request.pos4=target_z+0.24;
			msg.request.action_time=800;

			
			if(ros::Time::now().toSec()-secs>0.5)
			{
				servoseter.call(msg);
				secs=ros::Time::now().toSec();
			}
			
			// if (msg.response.is_done)printf("done: true\n");
			// else printf("done: false\n");
		}

		tag_watch_dog++;
		if (tag_watch_dog>2000)
		{
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



	
		
	
	
	return 0;
}
