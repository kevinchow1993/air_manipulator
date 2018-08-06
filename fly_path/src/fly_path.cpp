#include "ros/ros.h"
#include "ros/time.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <algorithm>  


#define pi 3.141592653589793


using namespace std;


/**
算法流程：根据最新接收到的路径点向量集，根据无人机位置反馈，
判断无人机当前离路径点中最近的下一个点，调用mavros的set_point飞向此点。 **/

class Flx_Path_wapper
{
	public:

	geometry_msgs::PoseStamped current_pose;
	nav_msgs::Path current_path;
	nav_msgs::Path last_path;
	geometry_msgs::PoseStamped current_destination_pose;
	int path_pointer;
	int is_path_update;
	int start_locker;
	nav_msgs::Path path_in_use;
	double distance_err;

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
		if(start_locker<100)
		{
			current_destination_pose= current_pose;//copy.deepcopy(current_pose);
			start_locker+=1;
		}
	}
	void  reload_path(nav_msgs::Path msg)
	{
		//current_path=msg.poses;
		current_path.poses.clear();
		for (int i = 0; i < msg.poses.size(); ++i)current_path.poses.push_back(msg.poses[i]);
		
	}
	//判断是否到达当前目标点
	int  is_meet_current_destination(void)
	{
	        double pose_err_x =fabs(current_pose.pose.position.x-path_in_use.poses[path_pointer].pose.position.x);
	        double pose_err_y=fabs(current_pose.pose.position.y-path_in_use.poses[path_pointer].pose.position.y);
	        double pose_err_z=fabs(current_pose.pose.position.z-path_in_use.poses[path_pointer].pose.position.z);
	        distance_err=sqrt(pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z);
	        if (distance_err<0.15)return 1;
	        else return 0;
	}
	//计算下一个目标点
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
//计算当前点和规划好的轨迹中哪一点最近
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
	//强制插补
	void Limit_destination_movement(double max_limit)
	{
		
			//compute err between cuurent poisition  and desination 
			double pose_err_x=fabs(current_pose.pose.position.x-current_destination_pose.pose.position.x);
			double pose_err_y=fabs(current_pose.pose.position.y-current_destination_pose.pose.position.y);
			double pose_err_z=fabs(current_pose.pose.position.z-current_destination_pose.pose.position.z);
			double total_err=pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z;
			double move_err=sqrt(pose_err_x*pose_err_x+pose_err_y*pose_err_y+pose_err_z*pose_err_z);

			if(move_err<=max_limit)return;

			double des_step=max_limit;
			double des_length=move_err;
			double delt_x=des_step/des_length*(current_destination_pose.pose.position.x-current_pose.pose.position.x);
			double delt_y=des_step/des_length*(current_destination_pose.pose.position.y-current_pose.pose.position.y);
			double delt_z=des_step/des_length*(current_destination_pose.pose.position.z-current_pose.pose.position.z);

			current_destination_pose.pose.position.x=current_pose.pose.position.x+delt_x;
			current_destination_pose.pose.position.y=current_pose.pose.position.y+delt_y;
			current_destination_pose.pose.position.z=current_pose.pose.position.z+delt_z;
		
	}

}Path_handler;



void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 Path_handler.reload_pose(msg);
}

void Path_Callback(nav_msgs::Path msg)
{
	 Path_handler.reload_path(msg);
	 ROS_INFO("RECEIVED A NEW PATH!");
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "fly_path");
  	ros::NodeHandle n;


  	ros::Subscriber Path_suber = n.subscribe("/flx_path_plan", 1, Path_Callback);
  	ros::Subscriber pose_feedback=n.subscribe("/mavros/vision_pose/pose",1,PoseStamped_Callback);
  	ros::Publisher set_point_puber = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
  	


  	ros::Rate loop_rate(50);
  	while (ros::ok())
	{
		//等待有效轨迹
		if (Path_handler.start_locker<100||Path_handler.current_path.poses.size()==0)ROS_INFO("wait for valid path");
		else
		{
			//计算下一个目标点
			Path_handler.compute_current_destination();
			Path_handler.current_destination_pose.header.stamp = ros::Time::now();
			Path_handler.current_destination_pose.header.frame_id = "fcu";
		        	unsigned int tt_Index=Path_handler.current_path.poses.size();
		        	printf("---------------\nPath done:(%d/%d),dr=%f\n",Path_handler.path_pointer+1,tt_Index,Path_handler.distance_err);
		        	printf("x:%.3lf  gx:%.3lf   e:%.3lf\n",Path_handler.current_pose.pose.position.x, Path_handler.current_destination_pose.pose.position.x,Path_handler.current_destination_pose.pose.position.x-Path_handler.current_pose.pose.position.x);
		        	printf("y:%.3lf  gy:%.3lf   e:%.3lf\n",Path_handler.current_pose.pose.position.y, Path_handler.current_destination_pose.pose.position.y,Path_handler.current_destination_pose.pose.position.y-Path_handler.current_pose.pose.position.y);
		        	printf("z:%.3lf  gz:%.3lf   e:%.3lf\n",Path_handler.current_pose.pose.position.z, Path_handler.current_destination_pose.pose.position.z,Path_handler.current_destination_pose.pose.position.z-Path_handler.current_pose.pose.position.z);
		        	Path_handler.Limit_destination_movement(0.2);
		        	
		        	double Zp=3.0,Zp_plus=0;
					//强制拔高
		        	if(Path_handler.current_destination_pose.pose.position.z>Path_handler.current_pose.pose.position.z)
		        	{
		        		Zp_plus=Zp*(Path_handler.current_destination_pose.pose.position.z-Path_handler.current_pose.pose.position.z);
		        		if(Zp_plus>=0.5)Zp_plus=0.5;
		        	}
		        	Path_handler.current_destination_pose.pose.position.z+=Zp_plus;

			set_point_puber.publish(Path_handler.current_destination_pose);//fly to current target position
			static tf::TransformBroadcaster br;
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(Path_handler.current_destination_pose.pose.position.x, Path_handler.current_destination_pose.pose.position.y, Path_handler.current_destination_pose.pose.position.z) );
			tf::Quaternion tf_q(
				Path_handler.current_destination_pose.pose.orientation.x,
				Path_handler.current_destination_pose.pose.orientation.y,
				Path_handler.current_destination_pose.pose.orientation.z,
				Path_handler.current_destination_pose.pose.orientation.w
				);
			transform.setRotation(tf_q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/des"));

		}
			

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}