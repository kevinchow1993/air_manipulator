#include "target_path_track.hpp"

using namespace std;
using namespace Eigen; 


target_path_track::target_path_track():
		last_pose(52.0+48.0),
		Dpose(45.0),
		tag_watch_dog(0),
		recent_marker_time(0.0),
		recent_target_time(0.0),
		current_interest_id(35),
		is_valid_track_target(false),
		TagLost_flag(false)
{
	init_Tlc();
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	m_serial.Open((char*)com_cs.c_str(), 9600, 8, NO, 1);
	Set_Servo_Pos(last_pose);
	ros::Duration(2.0).sleep();
	Camera_Pos_suber=n.subscribe("/camera_servo_pose",1,&target_path_track::CameraPos_CallBack,this);//call by aprilTags pack.
	Interest_ID_seter = n.advertiseService("/target_tracker/set_interest_id", &target_path_track::Set_interest_ID,this);
	Tba_puber=n.advertise<am_controller::Mat_Tba>("/Tba_current",1);
}

target_path_track::~target_path_track()
{
	m_serial.Close();
}

bool target_path_track::Set_interest_ID(camera_control::set_interest_ID::Request &msg,camera_control::set_interest_ID::Response &res)
{
	current_interest_id=msg.id;
	res.id_ret=current_interest_id;
	return true;
}

void target_path_track::Set_Servo_Pos(int pos)
{
	if (pos>150)pos=150;if (pos<43)pos=43;
	char cmd[100];
	sprintf(cmd,"%ds",pos);
	//printf("%ds\n",pos);
	m_serial.Write(cmd,strlen(cmd));
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

void target_path_track::set_Tbl_by_Dpose(void)
{
	double cosb=cos(Dpose/180.0*pi);
	double sinb=sin(Dpose/180.0*pi);
	Tbl(0,0)=cosb;		Tbl(0,1)=0.0;		Tbl(0,2)=sinb;		Tbl(0,3)=0.1218;	//从body到link的偏移量
	Tbl(1,0)=0.0;		Tbl(1,1)=1.0;		Tbl(1,2)=0.0;		Tbl(1,3)=0.0;
	Tbl(2,0)=-sinb;		Tbl(2,1)=0.0;		Tbl(2,2)=cosb;		Tbl(2,3)=-0.1105;
	Tbl(3,0)=0.0;		Tbl(3,1)=0.0;		Tbl(3,2)=0.0;		Tbl(3,3)=1.0;
}

void target_path_track::limited_Tba_puber(void)
{
	static double last_call_time=0;
	if(ros::Time::now().toSec()-last_call_time>0.01)
	{
		Tba_puber.publish(Tba_msg);	
		last_call_time=ros::Time::now().toSec();
	}
	
}

void target_path_track::target_detector(const geometry_msgs::PoseStamped &pose,int id)
{
	Quaterniond q(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
	Matrix3d Rca = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Oca(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
	Matrix4d Tca=MatrixXd::Zero(4,4);Tca(3,3)=1.0;
	Tca<<Rca;Tca.col(3)<<Oca;
	set_Tbl_by_Dpose();
	Matrix4d Tba;
	Tba=Tbl*Tlc*Tca;
	Tba_msg.r11=Tba(0,0);Tba_msg.r12=Tba(0,1);Tba_msg.r13=Tba(0,2);Tba_msg.t1=Tba(0,3);
	Tba_msg.r21=Tba(1,0);Tba_msg.r22=Tba(1,1);Tba_msg.r23=Tba(1,2);Tba_msg.t2=Tba(1,3);
	Tba_msg.r31=Tba(2,0);Tba_msg.r32=Tba(2,1);Tba_msg.r33=Tba(2,2);Tba_msg.t3=Tba(2,3);
	Tba_msg.id=id;
	
	Matrix3d M_rotation;
	M_rotation<<Tba;
	Quaterniond q_angles(M_rotation);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(Tba(0,3), Tba(1,3), Tba(2,3) ));
	tf::Quaternion tf_q(q_angles.x(),q_angles.y(),q_angles.z(),q_angles.w());
	transform.setRotation(tf_q);
	if(id==25)
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/body", "/grasp_target"));
	else if(id==35)
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/body", "/along_maker"));

}

void target_path_track::CameraPos_CallBack(const camera_control::CameraPos::ConstPtr &msg)
{
	if(msg->id!=25&&msg->id!=35)return;
	recent_target_time =ros::Time::now().toSec(); 
	is_valid_track_target=true;
	if(msg->id==25)current_interest_id=25;
	if(current_interest_id!=msg->id)
	{
		ROS_INFO("Interest ID is %d but current decated %d...",current_interest_id,msg->id);
		return;
	}
	target_detector(msg->pose,msg->id);
	ROS_INFO("---\nID:%d\nx:%lf \ny:%lf\nz:%lf\n",Tba_msg.id,Tba_msg.t1,Tba_msg.t2,Tba_msg.t3);

	tag_watch_dog=0;
	TagLost_flag=false;
	tag_center_x=msg->cx;
	tag_center_y=msg->cy;
	float d_err=fabs(tag_center_y-240.0);
	float d_p=0.023,delay_factor=0.0017;//d_p=0.053,delay_factor=0.0017;//
	if (tag_center_y>=250)last_pose-=(d_err*d_p);
	if (tag_center_y<230)last_pose+=(d_err*d_p);
	if (last_pose>52.0+96.0)last_pose=52.0+96.0;if (last_pose<52.0)last_pose=52.0; 
	Set_Servo_Pos(last_pose);
	ros::Duration(delay_factor*d_err*d_p).sleep();
}

void target_path_track::init_Tlc(void)
{
	//Tlc init
	Tlc=MatrixXd::Zero(4,4);Tlc(3,3)=1.0;
	Quaterniond qlc(-0.484592,0.533285,-0.517225,0.461797);//w x y z but cout<<qld.coffes()  is  x y z w.
	Matrix3d Rlc = qlc.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
	Vector3d Olc(0.0394777,-0.00845416,0.0742496);
	Tlc<<Rlc;Tlc.col(3)<<Olc;
}

void target_path_track::loop(void)
{
	ros::Rate loop_rate(1000);
	while(ros::ok())
	{
		if(is_valid_track_target)limited_Tba_puber();
		tag_watch_dog++;
		if (tag_watch_dog>80)is_valid_track_target=false;
		if (tag_watch_dog>2000)
		{
			is_valid_track_target=false;
			if(TagLost_flag==false)
			{
				TagLost_flag=true;
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
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"target_tracker");
	target_path_track flx_target_path;
	flx_target_path.loop();
	return 0;
}
