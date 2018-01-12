#include "am_controller/servoset_srv.h"
#include "ros/ros.h"


#define pi 3.141592653589793



int main(int argc,char** argv)
{
	if (argc==1)
	{
		printf("Input cmd:  0  for  read.\n");
		printf("Input cmd:  3 pos1 pos2 pos3 pos4  for  set.\n");
		return 0;
	}


// printf(   "You   have   inputed   total   %d   argments\n"   ,   argc   );  
 //   for( int  i=0   ;   i<argc   ;   i++)  
 //   {  
 //   printf(   "arg%d   :   %s\n"   ,   i   ,   argv[i]   );  
 //   }  
	ros::init(argc,argv,"servoset_seter");
	ros::NodeHandle n;
	//ros::Publisher ik_puber=n.advertise<serial_am::ikMsg>("/serial_am/Link_pose",1);
	ros::ServiceClient servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");


	am_controller::servoset_srv msg;

	 int cmd=atoi(argv[1]);
	 printf("input cmd:%d\n",cmd );
	switch(cmd)
	{
		case  1:
			msg.request.cmd=1;break;
		case 2:
			msg.request.cmd=2;
			msg.request.pos1=atof(argv[2]);
			msg.request.pos2=atof(argv[3]);
			msg.request.pos3=atof(argv[4]);
			msg.request.pos4=atof(argv[5]);
			msg.request.action_time=atoi(argv[6]);
			printf("set pos:%lf   %lf   %lf    %lf  atime:%d\n",msg.request.pos1,msg.request.pos2,msg.request.pos3,msg.request.pos4 ,msg.request.action_time);
			break;
		case 3:
			msg.request.cmd=3;
			msg.request.pos1=atof(argv[2]);
			msg.request.pos2=atof(argv[3]);
			msg.request.pos3=atof(argv[4]);
			msg.request.pos4=atof(argv[5]);
			printf("set pos:%lf   %lf   %lf    %lf\n",msg.request.pos1,msg.request.pos2,msg.request.pos3,msg.request.pos4 );
			break;
		case 4:
			msg.request.cmd=4;
			msg.request.action_time=atoi(argv[2]);
			break;
		case 5:
			msg.request.cmd=5;
			msg.request.pos1=atof(argv[2]);
			msg.request.pos2=atof(argv[3]);
			msg.request.pos3=atof(argv[4]);
			msg.request.pos4=atof(argv[5]);
			msg.request.action_time=atoi(argv[6]);
			printf("set kin_pos:%lf   %lf   %lf    %lf\n",msg.request.pos1,msg.request.pos2,msg.request.pos3,msg.request.pos4 );
			break;
		case 6:
			msg.request.cmd=6;
			msg.request.pos1=atof(argv[2]);
			msg.request.pos2=atof(argv[3]);
			msg.request.pos3=atof(argv[4]);
			msg.request.pos4=atof(argv[5]);
			msg.request.action_time=atoi(argv[6]);
			printf("kin_control_pos:%lf   %lf   %lf    %lf\n",msg.request.pos1,msg.request.pos2,msg.request.pos3,msg.request.pos4 );
			break;
		default :
			msg.request.cmd=99;
			break;
			

	}

	servoseter.call(msg);
	if (msg.response.is_done)printf("done: true\n");
	else printf("done: false\n");





	return 0;
}