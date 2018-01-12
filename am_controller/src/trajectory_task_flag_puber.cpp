#include "am_controller/servoset_srv.h"
#include "ros/ros.h"

#include <camera_control/do_trajectory_task.h>


int main(int argc,char** argv)
{
	ros::init(argc,argv,"trajectory_task_flag_puber");
	ros::NodeHandle n;

	ros::Publisher Trajectory_task_puber = n.advertise<camera_control::do_trajectory_task>("/trajectory_task_ready",1);//gererator path and pub

	ros::Rate loop_rate(50);
	while(ros::ok())
	{

		camera_control::do_trajectory_task msg;
		msg.do_trajectory_task=1;
		Trajectory_task_puber.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}