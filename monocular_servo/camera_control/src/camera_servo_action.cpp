#include "serial_am/ikSrv.h"
#include "serial_am/graper.h"
#include "serial_am/ikMsg.h"



#include <camera_control/Grasping.h>

#include "ros/ros.h"

#define pi 3.141592653589793

int is_ready,is_done;

serial_am::ikSrv msg;
serial_am::graper graper_action;
void ik_sub_Callback(serial_am::ikMsg src_msg)
{
	msg.request.theta_e=src_msg.theta_e;
	msg.request.theta4=src_msg.theta4;
	msg.request.px=src_msg.px;
	msg.request.py=src_msg.py;


	is_ready++;

}



int main(int argc,char** argv)
{
	ros::init(argc,argv,"control_puber");

// printf(   "You   have   inputed   total   %d   argments\n"   ,   argc   );  
 //   for( int  i=0   ;   i<argc   ;   i++)  
 //   {  
 //   printf(   "arg%d   :   %s\n"   ,   i   ,   argv[i]   );  
 //   }  

	ros::NodeHandle n;
	//ros::Publisher ik_puber=n.advertise<serial_am::ikMsg>("/serial_am/Link_pose",1);
	ros::ServiceClient ik_puber = n.serviceClient<serial_am::ikSrv>("/serial_am/Link_pose");
	ros::ServiceClient graper_puber = n.serviceClient<serial_am::graper>("/serial_am/graper");
	ros::Subscriber ik_suber = n.subscribe("/Camera_servo_avtion/msg", 1, ik_sub_Callback);

    ros::ServiceClient grasping_puber=n.serviceClient<camera_control::Grasping>("/Grasping_info");
    camera_control::Grasping Grasping_msg;



	ros::Rate loop_rate(5);

	msg.request.theta_e=0;
	msg.request.theta4=pi/2;
	msg.request.px=0.20;
	msg.request.py=-0.20;

	is_ready=0;
	is_done=0;

	while(ros::ok())
	{

		if (is_ready>2000)is_ready=2000;
		if (is_done>2000)is_done=2000;

		// ik_puber.call(msg);///test

		if (is_ready>5&&is_done<=10)
		{
					msg.request.px-=0.2;
					ik_puber.call(msg);
					if (msg.response.is_done)
					{
						is_done++;
						Grasping_msg.request.state=1;//tell fly node to hold;
						grasping_puber.call(Grasping_msg);
						//printf("Link done! Use %dnd solution.\n",msg.response.is_done);
					}
					else printf("Null solution!\n");
		}


		if (is_ready>5&&is_done>10&&is_done<30)
		{
					ik_puber.call(msg);
					if (msg.response.is_done)
					{
						is_done++;
						Grasping_msg.request.state=1;//tell fly node to hold;
						grasping_puber.call(Grasping_msg);
						//printf("Link done! Use %dnd solution.\n",msg.response.is_done);
					}
					else printf("Null solution!\n");
		}

		if (is_done==30)
		{
			is_done++;



			// serial_am::ikSrv init_msg;
			// init_msg.request.py=-300;
			// ik_puber.call(init_msg);

			Grasping_msg.request.state=2;//tell fly node to back home
			grasping_puber.call(Grasping_msg);

		}




		ros::spinOnce();
		loop_rate.sleep();
	}


	// serial_am::graper graper_action;
	// msg.request.theta_e=0;
	// msg.request.theta4=pi/2;
	// msg.request.px=0.20;
	// msg.request.py=-0.20;
	

	// ik_puber.call(msg);
	// if (msg.response.is_done)printf("Link done! Use %dnd solution.\n",msg.response.is_done);
	// else printf("Null solution!\n");

	// graper_action.request.state=0;//0  off 
	// graper_puber.call(graper_action);
	// if (graper_action.response.is_done)printf("graper off!\n");

	// ros::Duration(3.0).sleep();
	// graper_action.request.state=1;//1  on
	// graper_puber.call(graper_action);
	// if (graper_action.response.is_done)printf("graper on!\n");

	// bool t1=true;
	// ros::Rate loop_rate(1000);
	// while(ros::ok())
	// {
	// 	if (t1==true)
	// 	{
	// 		t1=false;
	// 		ik_puber.call(msg);
	// 		if (msg.response.is_done)
	// 		{
	// 			printf("done!\n");
	// 		}
	// 	}
		

	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }




	return 0;
}