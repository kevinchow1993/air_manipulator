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
#include <serial_am/control_state.h>

#include "serial_am/ikSrv.h"
#include "serial_am/graper.h"

#define pi 3.141592653589793


using namespace std;
enum offboard_state{Normal=1,offboard_turn_on=2,Landing=4,AM_back=8,offboard_turn_off=16};

class Flx_Path_wapper
{
	public:
	geometry_msgs::Point bang_center;
	geometry_msgs::PoseStamped current_pose;
	nav_msgs::Path current_path;
	nav_msgs::Path last_path;
	geometry_msgs::PoseStamped current_destination_pose;
	int path_pointer;
	int is_path_update;
	int start_locker;
	nav_msgs::Path path_in_use;
	double distance_err;
	uint8_t current_state;
	uint8_t last_state;
	uint8_t amlock;
	bool is_am_back;
	bool is_am_back_last;
	Flx_Path_wapper()
	{
		current_state=Normal;
		last_state=Normal;
		is_am_back_last=false;
		amlock=0;
	}
	~Flx_Path_wapper()
	{

	}
	void reload_state(serial_am::control_state msg)
	{
		if(msg.state&AM_back)is_am_back=true;
		else is_am_back=false;
		current_state=msg.state&(uint8_t)(~AM_back);
		if(amlock==0)amlock=msg.amlock;
	}
	void  reload_center(geometry_msgs::Point msg)
	{
		bang_center.x=msg.x;
		bang_center.y=msg.y;
		bang_center.z=msg.z;
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



}Path_handler;


void state_msg_Callback(serial_am::control_state msg)
{
	Path_handler.reload_state(msg);
}

void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	 Path_handler.reload_pose(msg);
}

void Path_Callback(nav_msgs::Path msg)
{
	 Path_handler.reload_path(msg);
}

void bang_center_Callback( geometry_msgs::Point msg)
{
	Path_handler.reload_center(msg);
}

void convert_world_to_body(geometry_msgs::Point &world, geometry_msgs::Point &body)//p_b=R_b_w*p_w   
{
	double x,y,z,w,xx,yy,zz,xy,wz,wy,xz,yz,wx,ox,oy,oz;//it isd from world to body
	ox=Path_handler.current_pose.pose.position.x;
	oy=Path_handler.current_pose.pose.position.y;
	oz=Path_handler.current_pose.pose.position.z-0.240;//link_offset
	x=Path_handler.current_pose.pose.orientation.x;
	y=Path_handler.current_pose.pose.orientation.y;
	z=Path_handler.current_pose.pose.orientation.z;
	w=Path_handler.current_pose.pose.orientation.w;
	xx=x*x;yy=y*y;zz=z*z;xy=x*y;wz=w*z;wy=w*y;xz=x*z;yz=y*z;wx=w*x;
	double R11,R12,R13,R21,R22,R23,R31,R32,R33;//R is R_w_b
	R11=1.0-2.0*(yy+zz);	R12=2.0*(xy-wz);		R13=2.0*(wy+xz);
	R21=2.0*(xy+wz);		R22=1.0-2.0*(xx+zz);	R23=2.0*(yz-wx);
	R31=2.0*(xz-wy);		R32=2.0*(yz+wx);		R33=1.0-2.0*(xx+yy);
	body.x=world.x*R11 + world.y*R21 + world.z*R31 - ox*R11 - oy*R21 - oz*R31;//have use R_w_b(conj to R_b_w)
	body.y=world.x*R12 + world.y*R22 + world.z*R32 - ox*R12 - oy*R22 - oz*R32;
	body.z=world.x*R13 + world.y*R23 + world.z*R33 - ox*R13 - oy*R23 - oz*R33;

	// body.x=world.x*R11 + world.y*R12 + world.z*R13;//use R_b_w
	// body.y=world.x*R21 + world.y*R22 + world.z*R23;
	// body.z=world.x*R31 + world.y*R32 + world.z*R33;


}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "fly_path");
  	ros::NodeHandle n;

  	ros::Subscriber control_state_suber=n.subscribe("/flx_control_state",10,state_msg_Callback);
  	ros::Subscriber bang_center_suber=n.subscribe("/bang_center",1,bang_center_Callback);
  	ros::Subscriber Path_suber = n.subscribe("/flx_path_plan", 1, Path_Callback);
  	ros::Subscriber pose_feedback=n.subscribe("/mavros/vision_pose/pose",1,PoseStamped_Callback);
  	ros::Publisher Path_puber = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
  	
  	//AM control publisher
  	ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	ros::ServiceClient graper_puber = n.serviceClient<serial_am::graper>("/serial_am/graper");

	int success_ik_counter=0;
  	ros::Rate loop_rate(50);
  	while (ros::ok())
	{

		if (Path_handler.current_state==Normal||Path_handler.current_state==offboard_turn_off)
		{
			Path_handler.current_destination_pose.header.stamp = ros::Time::now();
			Path_handler.current_destination_pose.header.frame_id = "fcu";
			Path_handler.current_destination_pose.pose.position.x = 0;//Path_handler.current_pose.pose.position.x;
			Path_handler.current_destination_pose.pose.position.y = 0;//Path_handler.current_pose.pose.position.y;
			Path_handler.current_destination_pose.pose.position.z = 0.34;
			Path_handler.current_destination_pose.pose.orientation.x = 0.0;//Path_handler.current_pose.pose.orientation.x;
			Path_handler.current_destination_pose.pose.orientation.y = 0.0;//Path_handler.current_pose.pose.orientation.y;
			Path_handler.current_destination_pose.pose.orientation.z = 0.0;//Path_handler.current_pose.pose.orientation.z;
			Path_handler.current_destination_pose.pose.orientation.w = -1.0;//Path_handler.current_pose.pose.orientation.w;
			Path_puber.publish(Path_handler.current_destination_pose);
		}
		if (Path_handler.current_state==offboard_turn_on)//Path_handler.current_state==Normal||
		{
			if (Path_handler.amlock){Path_handler.amlock=0;success_ik_counter=0;}
			if (success_ik_counter<=200)
			{
				if (Path_handler.last_state!=offboard_turn_on)
				{
					serial_am::ikSrv msg;//do_back_pos
					msg.request.py=-150;
					ik_puber.call(msg);
				}
				Path_handler.compute_current_destination();
		        Path_handler.current_destination_pose.header.stamp = ros::Time::now();
		        Path_handler.current_destination_pose.header.frame_id = "fcu";
		        if (Path_handler.start_locker<100)ROS_INFO("wait for valid path");
		        else
		        {
		        	unsigned int tt_Index=Path_handler.current_path.poses.size();
		        	printf("------\nPath done:(%d/%d),dr=%f\n",Path_handler.path_pointer+1,tt_Index,Path_handler.distance_err);
		            Path_puber.publish(Path_handler.current_destination_pose);//fly to current target position
		        }
			}
	        geometry_msgs::Point p_world,p_body;//world frame is NWU
	        p_world.x=Path_handler.bang_center.x;
	        p_world.y=Path_handler.bang_center.y;
	        p_world.z=Path_handler.bang_center.z;
	        convert_world_to_body(p_world,p_body);
	        double end_x,end_y,end_z;//convert from NWU to link_frame
	        end_x= p_body.x;
	        end_y= p_body.z;
	        end_z=-p_body.y;
	        printf("x=%f y=%f z=%f\n",end_x,end_y,end_z);

	        if (Path_handler.is_am_back)
	        {
	        	ROS_INFO("Am_back_Locker: On.");
	        	if (Path_handler.is_am_back_last==false)//first time to do it
				{
					Path_handler.amlock=0;
					serial_am::graper graper_action;
					graper_action.request.state=0;//0  off 
					graper_puber.call(graper_action);

					serial_am::ikSrv msg;//do_init_pos
					msg.request.py=-300;
					ik_puber.call(msg);
				}
	        }Path_handler.is_am_back_last=Path_handler.is_am_back;
	       
	        if (end_x<= 0.35&&end_x>= 0.10&&
	        	end_y<=-0.10&&end_y>=-0.25&&
	        	end_z<= 0.10&&end_z>=-0.10
	        	)
	        {
	        		if (success_ik_counter<10)
	        		{
	        			serial_am::ikSrv msg;
		        		msg.request.theta_e=0;
						msg.request.theta4=pi/2;
						msg.request.px=end_x;
						msg.request.py=end_y;
						if (Path_handler.is_am_back==0)ik_puber.call(msg);
						if (msg.response.is_done)
						{
							success_ik_counter++;
							//printf("Link done! Use %dnd solution.\n",msg.response.is_done);
						}
						else 
						{
							success_ik_counter=0;
							//printf("Null solution!\n");
						}
	        		}
	        		else //do grasp
	        		{
	        			
	        			if (success_ik_counter==11)
	        			{
	        				serial_am::graper graper_action;
							graper_action.request.state=1000;//delay 1000ms to auto grasp. 
							if (Path_handler.is_am_back==0)graper_puber.call(graper_action);
	        			}
	        			if (success_ik_counter>200)
	        			{
	        				//ought to have pick the bang. return.
	        				ROS_INFO("ought to have pick the bang. return.");
	        				Path_handler.current_destination_pose.header.stamp = ros::Time::now();
					        Path_handler.current_destination_pose.header.frame_id = "fcu";
					        Path_handler.current_destination_pose.pose.position.x = 0.0;
					        Path_handler.current_destination_pose.pose.position.y = 0.0;
					        Path_handler.current_destination_pose.pose.position.z = 0.6;
					        Path_puber.publish(Path_handler.current_destination_pose);


	        				success_ik_counter=222;
	        			}
	        			
						success_ik_counter++;
	        		}
	        		
	        }
		}
		else if (Path_handler.current_state==Landing)
		{
			ROS_INFO("Landing decated: AM_back and Landing...");
			if (Path_handler.amlock)//first time to do it
			{
				Path_handler.amlock=0;
				serial_am::graper graper_action;
				graper_action.request.state=0;//0  off 
				graper_puber.call(graper_action);

				serial_am::ikSrv msg;//do_init_pos
				msg.request.py=-300;
				ik_puber.call(msg);
			}
			Path_handler.current_destination_pose.header.stamp = ros::Time::now();
	        Path_handler.current_destination_pose.header.frame_id = "fcu";
	        Path_handler.current_destination_pose.pose.position.z = 0.3;
	        Path_puber.publish(Path_handler.current_destination_pose);

		}
		else if (Path_handler.current_state==offboard_turn_off)
		{
			ROS_INFO("offboard_turn_off decated: AM_back...");
			if (Path_handler.amlock)//first time to do it
			{
				Path_handler.amlock=0;
				serial_am::graper graper_action;
				graper_action.request.state=0;//0  off 
				graper_puber.call(graper_action);

				serial_am::ikSrv msg;//do_init_pos
				msg.request.py=-300;
				ik_puber.call(msg);
			}
		}
		Path_handler.last_state=Path_handler.current_state;


		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}