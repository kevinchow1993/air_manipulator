#include "ros/ros.h"
#include "std_msgs/String.h"
#include "am_controller/servoset_srv.h"

#include <iostream>
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "servo_calibriation");
	
	ros::NodeHandle n;
    ros::ServiceClient servo_calibriation = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
	am_controller::servoset_srv servo_msg;
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		
		for (int servo1 = 10; servo1 <170 ; servo1=servo1+10)
		{
			for (int servo2 = 10; servo2 < 170; servo2=servo2+10)
			{
				for (int servo3 = 10; servo3 < 170; servo3=servo3+10)
				{
					 servo_msg.request.cmd = 5;
					 servo_msg.request.pos1 = servo1;
					 servo_msg.request.pos2 = servo2;
					 servo_msg.request.pos3 = servo3;
					 servo_msg.request.pos4 = 0.19;
					 servo_msg.request.action_time = 2000;
					 servo_calibriation.call(servo_msg);
					 printf("set servo pose:%d	%d	%d	0.19\n",servo1,servo2,servo3);
					 while(servo_msg.response.is_done == false&ros::ok())
					 servo_msg.request.cmd = 1;
					 servo_calibriation.call(servo_msg);
					 printf("get servo pose:%lf	%lf	%lf	%lf\n",servo_msg.response.servo_pos1,
					 													 servo_msg.response.servo_pos2,
																		 servo_msg.response.servo_pos3,
																		 servo_msg.response.servo_pos4);
				}
				
			}
			
		}
		
		ros::spinOnce();
	}
	return 0;
}