#include "ros/ros.h"
#include "ros/time.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

#include <camera_control/Grasping.h>




using namespace std;


#define pi 3.141592653589793


	double vt_x,vt_y,vt_z,vt_t;
	double mav_x,mav_y,mav_z,mav_t,mav_yaw;
	int init_cnt1,init_cnt2;
	bool init_done;
	double qw,qx,qy,qz;
	nav_msgs::Path init_path;
	nav_msgs::Path back_path;

	bool is_landing;

int Grasping_state;

class Flx_Path_wapper
{
	public:
	geometry_msgs::PoseStamped current_pose;
	nav_msgs::Path current_path;
	geometry_msgs::PoseStamped current_destination_pose;
	int path_pointer;
	nav_msgs::Path path_in_use;
	double distance_err;
	nav_msgs::Path Landing_Path;

	Flx_Path_wapper()
	{

	}
	~Flx_Path_wapper()
	{

	}
	void reload_pose(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		current_pose.header.stamp = msg->header.stamp;
        current_pose.header.frame_id = msg->header.frame_id;
        current_pose.pose.orientation.x = msg->pose.orientation.x;
        current_pose.pose.orientation.y = msg->pose.orientation.y;
        current_pose.pose.orientation.z = msg->pose.orientation.z;
        current_pose.pose.orientation.w = msg->pose.orientation.w;
        current_pose.pose.position.x = msg->pose.position.x;
        current_pose.pose.position.y = msg->pose.position.y;
        current_pose.pose.position.z = msg->pose.position.z;
	}
	void  reload_path(nav_msgs::Path msg)
	{
		//current_path=msg.poses;
		current_path.poses.clear();
		for (int i = 0; i < msg.poses.size(); ++i)current_path.poses.push_back(msg.poses[i]);
		
	}
    int  is_meet_current_destination(void)
    {
        double pose_err_x =fabs(current_pose.pose.position.x-path_in_use.poses[path_pointer].pose.position.x);
        double pose_err_y=fabs(current_pose.pose.position.y-path_in_use.poses[path_pointer].pose.position.y);
        double pose_err_z=fabs(current_pose.pose.position.z-path_in_use.poses[path_pointer].pose.position.z);
        distance_err=sqrt(pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z);
        if (distance_err<0.15)return 1;
        else return 0;
    }
	void compute_current_destination(void)
	{
		if (path_in_use.poses.size()==0)//#first time to get taget
		{
			path_pointer=0;
			//path_in_use.poses=current_path.poses;
			path_in_use.poses.clear();
			for (int i = 0; i < current_path.poses.size(); ++i)path_in_use.poses.push_back(current_path.poses[i]);
		}
		else //#some path already in use, check if it should be changed
		{
			path_in_use.poses.clear();
			for (int i = 0; i < current_path.poses.size(); ++i)path_in_use.poses.push_back(current_path.poses[i]);
			path_pointer=get_point_in_path();
		}
		if  (path_pointer+1<path_in_use.poses.size())path_pointer+=is_meet_current_destination();
        if (path_in_use.poses.size())current_destination_pose.pose=path_in_use.poses[path_pointer].pose;

	}

	int get_point_in_path(void)
	{
		vector<double> err_list;
		for (int i = 0; i < path_in_use.poses.size(); ++i)
		{
			double pose_err_x=fabs(current_pose.pose.position.x-path_in_use.poses[i].pose.position.x);
            double pose_err_y=fabs(current_pose.pose.position.y-path_in_use.poses[i].pose.position.y);
            double pose_err_z=fabs(current_pose.pose.position.z-path_in_use.poses[i].pose.position.z);
            double total_err=pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z;
            err_list.push_back(total_err);
		}
		auto smallest=min_element(begin(err_list), end(err_list));
		return distance(begin(err_list), smallest);
	}
	void Make_landing_path(void)
	{
		Landing_Path.poses.clear();
		for (double i=current_pose.pose.position.z; i >0.1 ; i=i-0.1)
		{
				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = "fcu";
				pose.pose.orientation.x = qx;
				pose.pose.orientation.y = qy;
				pose.pose.orientation.z = qz;
				pose.pose.orientation.w = qw;
				pose.pose.position.x = current_pose.pose.position.x;
				pose.pose.position.y = current_pose.pose.position.y;
				pose.pose.position.z = i;
				Landing_Path.poses.push_back(pose);
		}
	}


}Path_handler;


	void func_pose_vitrualtag(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		// double roll, pitch, yaw;
		// tf::Quaternion q;
		// tf::quaternionMsgToTF(msg->pose.orientation, q);
	 	// tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	  	vt_x=msg->pose.position.x;
	  	vt_y=msg->pose.position.y;
	  	vt_z=msg->pose.position.z;
	  	vt_t=msg->header.stamp.toSec();

	  	init_cnt1++;
	}

	void Local_pose_CallBack(const geometry_msgs::PoseStampedConstPtr &msg)
	{

		Path_handler.reload_pose(msg);

		mav_x=msg->pose.position.x;
	  	mav_y=msg->pose.position.y;
	  	mav_z=msg->pose.position.z;
	  	mav_t=msg->header.stamp.toSec();

	  	double x,y,z,w,yaw;
		x=msg->pose.orientation.x;y=msg->pose.orientation.y;z=msg->pose.orientation.z;w=msg->pose.orientation.w;
		mav_yaw=atan2(2*w*z+2*x*y,1-2*y*y-2*z*z);
	  	init_cnt2++;
	}


	void init_easy_path(void)
		{
			if (init_cnt1<50||init_cnt2<50)return;
			double fai,theta,psai;
			psai=atan2(-mav_y+vt_y,-mav_x+vt_x);
			qw=cos(fai/2)*cos(theta/2)*cos(psai/2)+sin(fai/2)*sin(theta/2)*sin(psai/2);
			qx=sin(fai/2)*cos(theta/2)*cos(psai/2)-cos(fai/2)*sin(theta/2)*sin(psai/2);
			qy=cos(fai/2)*sin(theta/2)*cos(psai/2)+sin(fai/2)*cos(theta/2)*sin(psai/2);
			qz=cos(fai/2)*cos(theta/2)*sin(psai/2)-sin(fai/2)*sin(theta/2)*cos(psai/2);

			printf("yaw:%f\t%f\n", psai/pi*180.0,mav_yaw/pi*180.0);
			double dl=0.1,dy,dx;
			dx=dl*cos(psai);
			dy=dl*sin(psai);

			init_path.header.stamp = ros::Time::now();
			init_path.header.frame_id = "world";
			back_path.header.stamp = ros::Time::now();
			back_path.header.frame_id = "world";


			double path_length=sqrt((mav_x-vt_x)*(mav_x-vt_x)+(mav_y-vt_y)*(mav_y-vt_y));

			printf("path_length:%lf\n", path_length);

			for (int i=0; i*dl <path_length ; ++i)
			{
				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = "fcu";
				pose.pose.orientation.x = qx;
				pose.pose.orientation.y = qy;
				pose.pose.orientation.z = qz;
				pose.pose.orientation.w = qw;
				pose.pose.position.x = mav_x+i*dx;
				pose.pose.position.y = mav_y+i*dy;
				pose.pose.position.z = 0.8;
				init_path.poses.push_back(pose);
				printf("x,y   %f %f\n",pose.pose.position.x,pose.pose.position.y );
			}
			for (unsigned int i=init_path.poses.size()-1; i>0; --i)
			{
				back_path.poses.push_back(init_path.poses[i]);
			}
			init_done=true;

		}


bool Grasping_info_CallBack(camera_control::Grasping::Request &msg,camera_control::Grasping::Response &res)
{
	
	Grasping_state=msg.state;
	res.is_done=1;
	return true;
}


int main(int argc, char* argv[])
{

	ros::init(argc, argv, "flx_path_planner");
  	ros::NodeHandle n;

  	ros::Subscriber sub_pose_vitrualtag = n.subscribe("/vt/pose", 1, func_pose_vitrualtag);
  	ros::Subscriber local_pose_suber=n.subscribe("/mavros/vision_pose/pose",1,Local_pose_CallBack);///mavros/vision_pose/pose
  	ros::Publisher Path_puber = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
  	ros::ServiceServer Grasping_info_service = n.advertiseService("/Grasping_info", Grasping_info_CallBack);


  	init_cnt1=0;init_cnt2=0;
  	init_done=false;
  	is_landing=false;
  	init_path.poses.clear();
  	back_path.poses.clear();
  	Grasping_state=0;
  	bool is_force_back=false;
  	unsigned int tt_Index;

  	ros::Rate loop_rate(50);
  	while (ros::ok())
	{
		if (init_cnt1>1000)init_cnt1=1000;
		if (init_cnt2>1000)init_cnt2=1000;
		if(!init_done){init_easy_path();Path_handler.reload_path(init_path);}
		else
		{

			if (Grasping_state!=1)
			{
				Path_handler.compute_current_destination();
				Path_handler.current_destination_pose.header.stamp = ros::Time::now();
				Path_handler.current_destination_pose.header.frame_id = "fcu";
				tt_Index=Path_handler.current_path.poses.size();
				printf("------\nPath done:(%d/%d),dr=%f\n",Path_handler.path_pointer+1,tt_Index,Path_handler.distance_err);
				Path_puber.publish(Path_handler.current_destination_pose);//fly to current target position
			}
			else if (Grasping_state==1)//picking and  hold on
			{
				Path_puber.publish(Path_handler.current_destination_pose);
			}
			else if (Grasping_state==2)//fly back
			{
				Path_handler.reload_path(back_path);
			} 

			if (Grasping_state==2&&is_landing==false&&((Path_handler.path_pointer+1)/(float)tt_Index)>0.8)//landing
			{
				is_landing=true;
				Path_handler.Make_landing_path();
				Path_handler.reload_path(Path_handler.Landing_Path);
			}

			if (Grasping_state!=2&&is_landing==false&&((Path_handler.path_pointer+1)/(float)tt_Index)>0.7)//no track?   back home.
			{
				Path_handler.reload_path(back_path);
				is_force_back=true;
			}


			if (is_force_back==true&&is_landing==false&&((Path_handler.path_pointer+1)/(float)tt_Index)>0.8)//force back landing
			{
				is_landing=true;
				Path_handler.Make_landing_path();
				Path_handler.reload_path(Path_handler.Landing_Path);
			}



		}


		
		ros::spinOnce();
		loop_rate.sleep();
	}



	return 0;
}