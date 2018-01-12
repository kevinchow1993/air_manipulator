#include "ros/ros.h"
#include "ros/time.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <mavros_msgs/RCIn.h>
#include <camera_control/rc_state.h>

enum chan{c1,c2,c3,c4,c5,c6,c7,c8};
uint16_t channel[8]={0};

enum offboard_state{NORMAL=1,BACK_HOME,LADING};



void RC_read_Callback(mavros_msgs::RCIn RCmsg)
{

	for (int i = 0; i < RCmsg.channels.size(); ++i)
		channel[i]=RCmsg.channels[i];
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rc_state_node");
  	ros::NodeHandle n;
  	ros::Subscriber sub_pose_bang1 = n.subscribe("/mavros/rc/in", 1, RC_read_Callback);
  	ros::ServiceClient control_state_puber = n.serviceClient<camera_control::rc_state>("/flx_safe_state");

  	camera_control::rc_state control_msg;
  	control_msg.request.state=NORMAL;
  	uint8_t last_state=NORMAL;
  	// uint16_t last_c7=1000;
  	ros::Rate loop_rate(50);
  	while (ros::ok())
	{
		if(channel[c1])
		{
			//printf("%d %d %d %d %d %d %d %d\n",channel[c1],channel[c2],channel[c3],channel[c4],channel[c5],channel[c6],channel[c7],channel[c8]);
			if (channel[c8]<1200)control_msg.request.state=NORMAL;
			else if (channel[c8]>=1200&&channel[c8]<=1800)control_msg.request.state=BACK_HOME;//land
			else if (channel[c8]>1800)control_msg.request.state=LADING;//am always back

			if(control_msg.request.state!=last_state)
			{
				control_state_puber.call(control_msg);
				last_state=control_msg.request.state;
			}
			
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

 }