#ifndef TARGETPATHTRACK_H
#define TARGETPATHTRACK_H

#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include <camera_control/ServoPos.h>
#include <camera_control/CameraPos.h>
#include <camera_control/TagLost.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen> 

#include <geometry_msgs/PoseStamped.h>

#include <am_controller/Mat_Tba.h>

#include <camera_control/set_interest_ID.h>



#define pi  3.1415926535898

using namespace std;
using namespace Eigen; 

class target_path_track
{
	private:
		serial m_serial;
		float last_pose;
		float Dpose;
		double tag_center_x,tag_center_y;
		int tag_watch_dog;
		double recent_marker_time,recent_target_time;
		bool is_valid_track_target;
		int great_70_cnt;
		int current_interest_id;
		Matrix4d Twb;
		Matrix4d Tlc;
		Matrix4d Tbl;
		ros::NodeHandle n;
		std::string com_cs;
		ros::Publisher Tba_puber;
		ros::Subscriber Camera_Pos_suber;
		ros::ServiceServer Interest_ID_seter;
		am_controller::Mat_Tba Tba_msg;
		bool TagLost_flag;
	public:
		target_path_track();
		~target_path_track();
		bool Set_interest_ID(camera_control::set_interest_ID::Request &msg,camera_control::set_interest_ID::Response &res);
		void Set_Servo_Pos(int pos);
		void set_Tbl_by_Dpose(void);
		void limited_Tba_puber(void);
		void target_detector(const geometry_msgs::PoseStamped &pose,int id);
		void CameraPos_CallBack(const camera_control::CameraPos::ConstPtr &msg);
		void init_Tlc(void);
		void loop(void);

};












#endif