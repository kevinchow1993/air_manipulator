
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/CameraPos.h>
#include <camera_control/TagLost.h>
#include <camera_control/set_interest_ID.h>
#include <camera_control/World_pose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 



//#include "serial_am/ikMsg.h"
// #include "am_controller/servoset_srv.h"

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



// double secs;
double recent_marker_time,recent_target_time;
int current_interest_id;
Matrix4d Twb;

bool is_valid_track_marker;
bool is_valid_track_target;

int great_70_cnt;

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


bool Set_interest_ID(camera_control::set_interest_ID::Request &msg,camera_control::set_interest_ID::Response &res)
{
	current_interest_id=msg.id;
	res.id_ret=current_interest_id;
	return true;
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
	}
	else if(msg.id==35)
	{
		recent_marker_time =ros::Time::now().toSec(); 
		is_valid_track_marker=true;
	}
	if(current_interest_id!=msg.id)return;

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

void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg)
{

	  	Quaterniond q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
		Matrix3d Rwb = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
		Vector3d Owb(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
		Twb=MatrixXd::Zero(4,4);Twb(3,3)=1.0;
		Twb<<Rwb;Twb.col(3)<<Owb;

}

void land_marker_detector(Matrix4d &Tlc,ros::Publisher &id_world_pose_puber,tf::TransformListener &listener)
{
		if(is_valid_track_marker==false)return;
		tf::StampedTransform transform;
		bool valid_transform=false;
		try
		{
	    		if(ros::Time::now().toSec()-recent_marker_time<0.5)valid_transform=true;
			listener.lookupTransform("/camera", "/marker1", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
		    	valid_transform=false;
		    	//ROS_ERROR("%s",ex.what());
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
	    	
		    	Matrix4d Twa;
		    	Twa=Twb*Tba;

		 
		    	camera_control::World_pose msg;
		    	msg.bx=target_x;
		    	msg.by=target_y;
		    	msg.bz=target_z;
		    	msg.wx=Twa(0,3);
		    	msg.wy=Twa(1,3);
		    	msg.wz=Twa(2,3);
		    	msg.id=35;
		    	id_world_pose_puber.publish(msg);

			cout<<"--------marker_Info-----\n"<<msg.wx<<"\t\t"<<msg.wy<<"\t\t"<<msg.wz<<endl;
			//cout<<rol*180.0/pi<<"\t\t"<<pit*180.0/pi<<"\t\t"<<yaw*180.0/pi<<endl;

			M_rotation<<Twa;
			Quaterniond q_wa(M_rotation);
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(msg.wx, msg.wy, msg.wz) );
			tf::Quaternion tf_q(q_wa.x(),q_wa.y(),q_wa.z(),q_wa.w());
			transform.setRotation(tf_q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/grasp_marker"));


	
		}
}

void target_detector(Matrix4d &Tlc,ros::Publisher &id_world_pose_puber,tf::TransformListener &listener)
{
		if(is_valid_track_target==false)return;
		tf::StampedTransform transform;
		bool valid_transform=false;
		try
		{
	    		if(ros::Time::now().toSec()-recent_marker_time<0.5)valid_transform=true;
			listener.lookupTransform("/camera", "/target1", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
		    	valid_transform=false;
		    	//ROS_ERROR("%s",ex.what());
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
	    	
		    	Matrix4d Twa;
		    	Twa=Twb*Tba;


		    	camera_control::World_pose msg;
		    	msg.bx=target_x+0.04;
		    	msg.by=target_y;
		    	msg.bz=target_z+0.24;
		    	msg.wx=Twa(0,3);
		    	msg.wy=Twa(1,3);
		    	msg.wz=Twa(2,3);
		    	msg.id=25;
		    	id_world_pose_puber.publish(msg);

		    	cout<<"--------target_Info-----\n"<<msg.wx<<"\t\t"<<msg.wy<<"\t\t"<<msg.wz<<endl;
			//cout<<"--------tag_Info-----\n"<<target_x<<"\t\t"<<target_y<<"\t\t"<<target_z<<endl;
			//cout<<rol*180.0/pi<<"\t\t"<<pit*180.0/pi<<"\t\t"<<yaw*180.0/pi<<endl;
		    	M_rotation<<Twa;
			Quaterniond q_wa(M_rotation);
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(msg.wx, msg.wy, msg.wz) );
			tf::Quaternion tf_q(q_wa.x(),q_wa.y(),q_wa.z(),q_wa.w());
			transform.setRotation(tf_q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/grasp_target"));


		}
}

int main(int argc, char** argv)
{
	//test_fun();

	ros::init(argc,argv,"target_tracker");
	ros::NodeHandle n;
	std::string com_cs;
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);
	Set_Servo_Pos(last_pose);


	ros::Duration(2.0).sleep();
	ros::Subscriber Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,CameraPos_CallBack);//call by aprilTags pack.
	ros::ServiceClient tag_lost_puber=n.serviceClient<camera_control::TagLost>("/TagLost");

	// ros::ServiceClient id_world_pose_puber = n.serviceClient<camera_control::World_pose>("/target_tracker/target_world_pose");
	ros::Publisher id_world_pose_puber = n.advertise<camera_control::World_pose>("/target_tracker/target_world_pose",1);




	ros::ServiceServer Interest_ID_seter = n.advertiseService("/target_tracker/set_interest_id", Set_interest_ID);
	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose

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

	// ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
	// secs =ros::Time::now().toSec(); 
	recent_marker_time=0;
	recent_target_time=0;
	current_interest_id=35;

	tf::TransformListener listener;


	while(ros::ok())
	{
		if(current_interest_id==35)
			land_marker_detector(Tlc,id_world_pose_puber,listener);
		else if(current_interest_id==25)
			target_detector(Tlc,id_world_pose_puber,listener);
		

		tag_watch_dog++;
		if (tag_watch_dog>80)
		{
			is_valid_track_marker=false;
			is_valid_track_target=false;
		}
		if (tag_watch_dog>2000)
		{
			is_valid_track_marker=false;
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



	
		
	
	
	return 0;
}
