
#include "ros/ros.h"
#include <stdio.h>
#include "serial.h"
#include "am_controller/servoset_srv.h"
#include "Kin.h"
#include <fstream>
#include <iostream>

#define make_state_recorder

#ifdef make_state_recorder
	ofstream state_file_recorder;		
	double first_time=0;
#endif
	
	




// Create serial port
serial serial;
using namespace std;

typedef  unsigned char byte;
byte getChecksum(byte length,byte cmd,byte mydata[]){
    byte checksum=0;
    checksum ^= (length&0xFF);
    checksum ^= (cmd&0xFF);
    for(int i=0;i<length;i++)
    {
        checksum ^= (mydata[i]&0xFF);
    }
    return checksum;
}

void timerCallback(const ros::TimerEvent&)
{

}


bool get_current_servopos(double &servo_pos1,double &servo_pos2,double &servo_pos3,double &servo_pos4)
{
	byte buffer_length;
	byte cmd_buffer[200]={0x24,0x4D,0x3E};//$M>
	cmd_buffer[3]=0;//date length.
	cmd_buffer[4]=1;//cmd code;
	buffer_length=6+cmd_buffer[3];
	cmd_buffer[buffer_length-1]=getChecksum(cmd_buffer[3],cmd_buffer[4],cmd_buffer+5);


	for (int i = 0; i <buffer_length ; ++i)
		printf("%02x ", (int)cmd_buffer[i]);
	printf("\n");


	serial.Write((char*)cmd_buffer,buffer_length);
	//ros::Duration(0.0001);//delay 0.1ms
	byte read_buffer[256]={0};
	char buffer[256];
	int total_length=0;
	double t_begin = ros::Time::now().toSec();
	while(ros::Time::now().toSec()-t_begin<0.05)
	{
		int length_read=serial.Read(buffer);
		for (int i = 0; i <length_read ; ++i)read_buffer[i+total_length]=buffer[i];
		total_length+=length_read;
		//check valid cmd header
		int valid_length=14;
		if (total_length==valid_length)
		{
			if(read_buffer[0]==0x24&&read_buffer[1]==0x4D&&read_buffer[2]==0x3C&&read_buffer[valid_length-1]==getChecksum(read_buffer[3],read_buffer[4],read_buffer+5))
			{
				

				unsigned short s_servo_pos1,s_servo_pos2,s_servo_pos3,s_servo_pos4;
				s_servo_pos1=(unsigned short)read_buffer[5]*256+read_buffer[6];
				s_servo_pos2=(unsigned short)read_buffer[7]*256+read_buffer[8];
				s_servo_pos3=(unsigned short)read_buffer[9]*256+read_buffer[10];
				s_servo_pos4=(unsigned short)read_buffer[11]*256+read_buffer[12];
				servo_pos1=s_servo_pos1/10.0;
				servo_pos2=s_servo_pos2/10.0;
				servo_pos3=s_servo_pos3/10.0;
				servo_pos4=s_servo_pos4/10.0;
				return true;
			}
			else
			{
				// for (int i = 0; i <valid_length ; ++i)
				// 	printf("%02x ", (int)read_buffer[i]);
				// printf("\n");
				//wrong cmd.  flush the serial;
				while(serial.Read(buffer));
				printf("wrong cmd .      exit code:1\n" );
				return false;
			}
		}
		else if(total_length>valid_length)
		{
			//wrong cmd.  flush the serial;
			while(serial.Read(buffer));
			printf("wrong cmd .      exit code:2\n" );
			return false;
		}
	}
	//time out.  flush the serial;
	while(serial.Read(buffer));
	printf("get cmd  time out.      exit code:3\n" );
	return false;
}


void set_servopos(double servo_pos1,double servo_pos2,double servo_pos3,double servo_pos4)
{
	byte buffer_length;
	unsigned short s_servo_pos1,s_servo_pos2,s_servo_pos3,s_servo_pos4;
	byte cmd_buffer[200]={0x24,0x4D,0x3E};//$M>
	cmd_buffer[3]=8;//date length.
	cmd_buffer[4]=3;//cmd code;
	s_servo_pos1=(unsigned short)(servo_pos1*10);s_servo_pos2=(unsigned short)(servo_pos2*10);s_servo_pos3=(unsigned short)(servo_pos3*10);s_servo_pos4=(unsigned short)(servo_pos4*10);
	cmd_buffer[5]	=s_servo_pos1/256;	cmd_buffer[6]	=s_servo_pos1%256;
	cmd_buffer[7]	=s_servo_pos2/256;	cmd_buffer[8]	=s_servo_pos2%256;
	cmd_buffer[9]	=s_servo_pos3/256;	cmd_buffer[10]	=s_servo_pos3%256;
	cmd_buffer[11]	=s_servo_pos4/256;	cmd_buffer[12]	=s_servo_pos4%256;
	buffer_length=6+cmd_buffer[3];
	cmd_buffer[buffer_length-1]=getChecksum(cmd_buffer[3],cmd_buffer[4],cmd_buffer+5);

	for (int i = 0; i <buffer_length ; ++i)
		printf("%02x ", (int)cmd_buffer[i]);
	printf("\n");

	serial.Write((char*)cmd_buffer,buffer_length);
}

void set_servopos_t(double servo_pos1,double servo_pos2,double servo_pos3,double servo_pos4,unsigned int action_time)
{
	if (servo_pos1<0||servo_pos2<0||servo_pos3<0||servo_pos4<0)
	{
		printf("err pos by nagetive.%lf %lf %lf %lf\n",servo_pos1,servo_pos2,servo_pos3,servo_pos4);
		return ;
	}
	byte buffer_length;
	unsigned short s_servo_pos1,s_servo_pos2,s_servo_pos3,s_servo_pos4;
	byte cmd_buffer[200]={0x24,0x4D,0x3E};//$M>
	cmd_buffer[3]=10;//date length.
	cmd_buffer[4]=2;//cmd code;
	s_servo_pos1=(unsigned short)(servo_pos1*10);s_servo_pos2=(unsigned short)(servo_pos2*10);s_servo_pos3=(unsigned short)(servo_pos3*10);s_servo_pos4=(unsigned short)(servo_pos4*10);
	cmd_buffer[5]	=s_servo_pos1/256;	cmd_buffer[6]	=s_servo_pos1%256;
	cmd_buffer[7]	=s_servo_pos2/256;	cmd_buffer[8]	=s_servo_pos2%256;
	cmd_buffer[9]	=s_servo_pos3/256;	cmd_buffer[10]	=s_servo_pos3%256;
	cmd_buffer[11]	=s_servo_pos4/256;	cmd_buffer[12]	=s_servo_pos4%256;
	cmd_buffer[13]	=action_time/256;	cmd_buffer[14]	=action_time%256;
	buffer_length=6+cmd_buffer[3];
	cmd_buffer[buffer_length-1]=getChecksum(cmd_buffer[3],cmd_buffer[4],cmd_buffer+5);

	for (int i = 0; i <buffer_length ; ++i)
		printf("%02x ", (int)cmd_buffer[i]);
	printf("\n");

	serial.Write((char*)cmd_buffer,buffer_length);
}

void grasp_action(unsigned int action_time)
{
	byte buffer_length;
	byte cmd_buffer[200]={0x24,0x4D,0x3E};//$M>
	cmd_buffer[3]=1;//date length.
	cmd_buffer[4]=4;//cmd code;
	cmd_buffer[5]	=action_time;
	buffer_length=6+cmd_buffer[3];
	cmd_buffer[buffer_length-1]=getChecksum(cmd_buffer[3],cmd_buffer[4],cmd_buffer+5);

	for (int i = 0; i <buffer_length ; ++i)
		printf("%02x ", (int)cmd_buffer[i]);
	printf("\n");

	serial.Write((char*)cmd_buffer,buffer_length);
}

void set_servopos_mono(double servo_pos,byte serv_ID)
{
	switch(serv_ID)
	{
		case 1:set_servopos(servo_pos,200.0,200.0,200.0);break;
		case 2:set_servopos(200.0,servo_pos,200.0,200.0);break;
		case 3:set_servopos(200.0,200.0,servo_pos,200.0);break;
		case 4:set_servopos(200.0,200.0,200.0,servo_pos);break;
		default:break;
	}
}

void set_kinpos_t(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4,unsigned int action_time)
{
	while(joint_pos1>180)joint_pos1-=180;
	while(joint_pos2>180)joint_pos2-=180;
	while(joint_pos3>180)joint_pos3-=180;
	while(joint_pos4>180)joint_pos4-=180;
	#ifdef make_state_recorder
		if(first_time==0)first_time=ros::Time::now().toSec();
		state_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<< ros::Time::now().toSec()-first_time<<" "<<joint_pos1/360*2.0*pi<<" "<<joint_pos2/360*2.0*pi<<" "<<joint_pos3/360*2.0*pi<<" "<<joint_pos4/360*2.0*pi<<" "<<action_time<<endl;
	#endif
	FLX_kinematics Kin; 
	Kin.Forward_Kinematics(joint_pos1/360*2.0*pi,joint_pos2/360.0*2.0*pi,joint_pos3/360.0*2.0*pi,joint_pos4/360.0*2.0*pi);
	Kin.Show_Forward_Result_RT();

	double servo_pos1=5.0-joint_pos1;
	double servo_pos2=1.2*joint_pos2-25.0;
	double servo_pos3=-1.1*joint_pos2 -0.64444*joint_pos3+207;
	double servo_pos4=joint_pos4+22;

	printf("set pos:%lf   %lf   %lf    %lf  atime:%d\n",servo_pos1,servo_pos2,servo_pos3,servo_pos4,action_time );
	set_servopos_t(servo_pos1,servo_pos2,servo_pos3,servo_pos4,action_time);
}

int Valid_Space(FLX_kinematics &Kin)
{

	for (int i = 0; i < 2; ++i)
	{
		// double joint_pos1=Kin.theta[i].theta1*180.0/pi;
		// double joint_pos2=Kin.theta[i].theta2*180.0/pi;
		// double joint_pos3=Kin.theta[i].theta3*180.0/pi;
		// double joint_pos4=Kin.theta[i].theta4*180.0/pi;
		// printf("test:%lf %lf %lf %lf\n",joint_pos1,joint_pos2,joint_pos3,joint_pos4 );
		if (
				(Kin.theta[i].theta1<5.0*pi/180.0)&&(Kin.theta[i].theta1>-150.0*pi/180.0)&&
				(Kin.theta[i].theta2>0)&&(Kin.theta[i].theta2<pi)//&&
				//(Kin->theta[i].theta3)&&(Kin->theta[i].theta3)
			)return i+1;
	}
	return 0;
	
}
bool  Valid_Angle(double joint_pos1,double joint_pos2,double joint_pos3,double joint_pos4)
{
	// double servo_pos1=5.0-joint_pos1;
	// double servo_pos2=1.2*joint_pos2-25.0;
	// double servo_pos3=-1.1*joint_pos2 -0.64444*joint_pos3+207;
	// double servo_pos4=joint_pos4+2;
	// if (servo_pos1<0||servo_pos2<0||servo_pos3<0||servo_pos4<0||servo_pos1>180||servo_pos2>180||servo_pos3>180||servo_pos4>180)
	// {
	// 	printf("+++++++++++++err pos :  %lf %lf %lf %lf\n",servo_pos1,servo_pos2,servo_pos3,servo_pos4);
	// 	return false;
	// }
	// else return true;
	return FLX_kinematics::Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4);
}

bool Kinematic_control(double theta_e,double theta4,double px,double py,unsigned int action_time)
{
	FLX_kinematics Kin; 
	Kin.Inverse_Kinematics(	theta_e,theta4,px,py);
	bool solution_exist=true;
	if (Kin.theta.size())
	{
		int valid_n=Valid_Space(Kin);
		if (valid_n>0)
		{
			Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
			double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
			joint_pos1=Kin.theta[valid_n-1].theta1*180.0/pi;
			joint_pos2=Kin.theta[valid_n-1].theta2*180.0/pi;
			joint_pos3=Kin.theta[valid_n-1].theta3*180.0/pi;
			joint_pos4=Kin.theta[valid_n-1].theta4*180.0/pi;

			while(joint_pos1>180)joint_pos1-=180;
			while(joint_pos2>180)joint_pos2-=180;
			while(joint_pos3>180)joint_pos3-=180;
			while(joint_pos4>180)joint_pos4-=180;


			if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
			{
				set_kinpos_t(joint_pos1,joint_pos2,joint_pos3,joint_pos4,action_time);
			}
			else
			{
				solution_exist=false;
				printf("Null Angle.\n");
			}
		}
		else 
		{
			solution_exist=false;
			printf("Null solution a.\n");
		}
	}
	else 
	{
		solution_exist=false;
		printf("Null solution b.\n");
	}
	return solution_exist;
}

bool Kinematic_valid(double theta_e,double theta4,double px,double py,unsigned int action_time)
{
	FLX_kinematics Kin; 
	Kin.Inverse_Kinematics(	theta_e,theta4,px,py);
	bool solution_exist=true;
	if (Kin.theta.size())
	{
		int valid_n=Valid_Space(Kin);
		if (valid_n>0)
		{
			Kin.Show_Inverse_Result_Theta(Kin.theta[valid_n-1]);
			double joint_pos1, joint_pos2, joint_pos3, joint_pos4;
			joint_pos1=Kin.theta[valid_n-1].theta1*180.0/pi;
			joint_pos2=Kin.theta[valid_n-1].theta2*180.0/pi;
			joint_pos3=Kin.theta[valid_n-1].theta3*180.0/pi;
			joint_pos4=Kin.theta[valid_n-1].theta4*180.0/pi;

			while(joint_pos1>180)joint_pos1-=180;
			while(joint_pos2>180)joint_pos2-=180;
			while(joint_pos3>180)joint_pos3-=180;
			while(joint_pos4>180)joint_pos4-=180;


			if(Valid_Angle(joint_pos1,joint_pos2,joint_pos3,joint_pos4))
			{
				//set_kinpos_t(joint_pos1,joint_pos2,joint_pos3,joint_pos4,action_time);
			}
			else
			{
				solution_exist=false;
				printf("Null Angle.\n");
			}
		}
		else 
		{
			solution_exist=false;
			printf("Null solution a.\n");
		}
	}
	else 
	{
		solution_exist=false;
		printf("Null solution b.\n");
	}
	return solution_exist;
}

bool pos_serv_CallBack(am_controller::servoset_srv::Request &msg,am_controller::servoset_srv::Response &res)//注意：service阻塞
{
	//double servo_pos1,servo_pos2,servo_pos3,servo_pos4;

	printf("I revice cmd: %d\n",msg.cmd );
	res.is_done=0;
	switch(msg.cmd)
	{
		case 1:
			printf("try get pos\n" );
			 while(get_current_servopos(res.servo_pos1,res.servo_pos2,res.servo_pos3,res.servo_pos4)==false);
			 printf("got pos:%lf   %lf   %lf    %lf \n",res.servo_pos1,res.servo_pos2,res.servo_pos3,res.servo_pos4 );
			 res.is_done=1;
			 break;
		case 2:
			printf("try set pos action_time\n" );
			set_servopos_t(msg.pos1,msg.pos2,msg.pos3,msg.pos4 ,msg.action_time);
			printf("set pos:%lf   %lf   %lf    %lf  atime:%d\n",msg.pos1,msg.pos2,msg.pos3,msg.pos4,msg.action_time );
			res.is_done=1;
			break;
		case 3:
			printf("try set_servopos\n" );
			set_servopos(msg.pos1,msg.pos2,msg.pos3,msg.pos4 );
			printf("set_servopos:%lf   %lf   %lf    %lf\n",msg.pos1,msg.pos2,msg.pos3,msg.pos4 );
			res.is_done=1;
			break;
		case 4:
			printf("try grasp action\n" );
			grasp_action(msg.action_time );
			res.is_done=1;
			break;
		case 5:
			printf("try set_kin_agnle_t\n" );
			set_kinpos_t(msg.pos1,msg.pos2,msg.pos3,msg.pos4 ,msg.action_time);
			printf("set set_kin_agnle_t:%lf   %lf   %lf    %lf  atime:%d\n",msg.pos1,msg.pos2,msg.pos3,msg.pos4,msg.action_time );
			res.is_done=1;
			break;
		case 6:
			printf("try kin_control\n" );
			res.is_done=Kinematic_control(msg.pos1,msg.pos2,msg.pos3,msg.pos4 ,msg.action_time);
			break;
		case 7:
			printf("is kin_valid\n" );
			res.is_done=Kinematic_valid(msg.pos1,msg.pos2,msg.pos3,msg.pos4 ,msg.action_time);
			break;
		default:
			printf("NULL cmd.\n");
			res.is_done=0;
			break;
	}
	
	return true;
}





int main(int argc, char** argv)
{

	ros::init(argc,argv,"serial_ros_flx");
	ROS_INFO("path:%s",argv[0]);
	ros::NodeHandle n;
	std::string com_cs;
	n.param<std::string>("COM_dev",com_cs,"/dev/ttyUSB_CP210X");
	serial.Open((char*)com_cs.c_str(), 115200, 8, NO, 1);
	
	/*#ifdef make_state_recorder
		state_file_recorder.open("/home/flx/catkin_al/state_recorder.txt",ios::out|ios::trunc);
		if(state_file_recorder==NULL)ROS_INFO("NULL FILE PTR");		
	#endif*/

	ros::Timer timer = n.createTimer(ros::Duration(0.002), timerCallback);
	ros::ServiceServer pos_service = n.advertiseService("/am_controller/servoset_srv", pos_serv_CallBack);

	ros::Duration(2.0).sleep();
	set_kinpos_t(-10,170,-70,0,3000);
	ros::MultiThreadedSpinner s(4);
  	ros::spin(s);

	// ros::Rate loop_rate(1000);
	// while(ros::ok())
	// {
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }

	serial.Close();
	#ifdef make_state_recorder
		state_file_recorder.close();
	#endif
	
	return 0;
}
