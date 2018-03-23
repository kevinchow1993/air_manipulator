
#include <trajectory_controller.hpp>
#include <fstream>

using namespace std;
using namespace Eigen; 

#define make_control_recorder

#ifdef make_control_recorder
  ofstream target_file_recorder;   
   ofstream correct_trajectory_file_recorder;   
  double first_time=0;
#endif


  trajectory_controller::trajectory_controller():
  ac("/trajectory_nsga2_generator", true),
  time_Tbar(0.0),
  task_rev(false)
  {
    trajectory_success_flag_puber=n.advertise<am_controller::success_flag>("/trajectory_follow_success_flag",1);
    Tba_suber=n.subscribe("/Tba_current",1,&trajectory_controller::Tba_callback,this);
    Task_suber=n.subscribe("/trajectory_task_ready",1,&trajectory_controller::Task_callback,this);
    servoseter = n.serviceClient<am_controller::servoset_srv>("/am_controller/servoset_srv");
    init_Tmb();
  }

  trajectory_controller::~trajectory_controller()
  {

  }

 void trajectory_controller::Task_callback(const camera_control::do_trajectory_taskConstPtr &msg)
 {
    if(msg->do_trajectory_task==1)task_rev=true;
 }

void trajectory_controller::init_Tmb()
{
  //Tmb init
  Tmb=MatrixXd::Zero(4,4);Tmb(3,3)=1.0;
  Tmb(0,0)=1.0;   Tmb(0,1)=0.0;   Tmb(0,2)=0.0;   Tmb(0,3)=0.07;  //从manipulator到body   0.04
  Tmb(1,0)=0.0;   Tmb(1,1)=0.0;   Tmb(1,2)=1.0;   Tmb(1,3)=0.24;
  Tmb(2,0)=0.0;   Tmb(2,1)=-1.0;    Tmb(2,2)=0.0;   Tmb(2,3)=0.0;
  Tmb(3,0)=0.0;   Tmb(3,1)=0.0;   Tmb(3,2)=0.0;   Tmb(3,3)=1.0;
}

  void trajectory_controller::set_ab(const vector<double> &para,const double &t_a_,const double &t_b_)
  {
       int cnt=0;
      for(int i=0;i<3;i++)
        for(int j=0;j<6;j++)
          link[i].a[j]=para[cnt++],link[i].b[j]=para[cnt++];
        t_a=t_a_;
        t_b=t_b_;
  }
  Matrix4d trajectory_controller::get_inverse_SE3(Matrix4d M12)
  {
    Matrix3d R12;R12<<M12;
    Vector3d O12;O12<<M12.col(3);
    Matrix4d M21;M21<<M12;
    Matrix3d R21;R21=R12.transpose();
    Vector3d O21;O21=-R21*O12;
    M21<<R21;
    M21.col(3)<<O21;
    return M21;
  }
  double trajectory_controller::get_link_angle_a(int link_no,double t){return link[link_no].a[0]+link[link_no].a[1]*t+link[link_no].a[2]*t*t+link[link_no].a[3]*t*t*t+link[link_no].a[4]*t*t*t*t+link[link_no].a[5]*t*t*t*t*t;}
  double trajectory_controller::get_link_angle_b(int link_no,double t){return link[link_no].b[0]+link[link_no].b[1]*t+link[link_no].b[2]*t*t+link[link_no].b[3]*t*t*t+link[link_no].b[4]*t*t*t*t+link[link_no].b[5]*t*t*t*t*t;}
  double trajectory_controller::get_link_vel_a(int link_no,double t){return link[link_no].a[1]+2*link[link_no].a[2]*t+3*link[link_no].a[3]*t*t+4*link[link_no].a[4]*t*t*t+5*link[link_no].a[5]*t*t*t*t;}
  double trajectory_controller::get_link_vel_b(int link_no,double t){return link[link_no].b[1]+2*link[link_no].b[2]*t+3*link[link_no].b[3]*t*t+4*link[link_no].b[4]*t*t*t+5*link[link_no].b[5]*t*t*t*t;}
  double trajectory_controller::get_link_acc_a(int link_no,double t){return 2*link[link_no].a[2]+6*link[link_no].a[3]*t+12*link[link_no].a[4]*t*t+20*link[link_no].a[5]*t*t*t;}
  double trajectory_controller::get_link_acc_b(int link_no,double t){return 2*link[link_no].b[2]+6*link[link_no].b[3]*t+12*link[link_no].b[4]*t*t+20*link[link_no].b[5]*t*t*t;}
  double trajectory_controller::get_link_angle(int link_no,double t)
  {
    if(t<=t_a)return get_link_angle_a(link_no,t);
    else if(t<=t_a+t_b)return get_link_angle_b(link_no,t-t_a);
    else return get_link_angle_b(link_no,t_b);
  }
  void trajectory_controller::get_prepare_manipulator_pose(double t,Matrix4d &result_SE3)
  {
    Kin.Forward_Kinematics(get_link_angle(0,t),get_link_angle(1,t),get_link_angle(2,t),0.0,result_SE3);
    //Kin.Show_Forward_Result_RT();
  }
  void trajectory_controller::get_correctional_manipulator_pose(double t,Matrix4d &result_SE3)//Tmer
  {

     #ifdef make_control_recorder
        Matrix4d Tma=Tmb*Tbar;
       double target_x,target_y,target_z;
       target_x=Tma(0,3);target_y=Tma(1,3);target_z=Tma(2,3);
         target_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<<t<<" "<<target_x<<" "<<target_y<<endl;
      #endif
  



    Matrix4d Tme;
    get_prepare_manipulator_pose(t,Tme);//get Tme with t.
    Matrix4d Tem=get_inverse_SE3(Tme);

    Matrix4d Tea=Tem*Tmb*Tba;
    //result_SE3:   Tmer=Tmb*Tbar*Tare;
    Matrix4d Tae=get_inverse_SE3(Tea);
    result_SE3=Tmb*Tbar*Tae;
  }


void trajectory_controller::Tba_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg)
{

   
      Tbar(0,0)=Tba_msg->r11;Tbar(0,1)=Tba_msg->r12;Tbar(0,2)=Tba_msg->r13;Tbar(0,3)=Tba_msg->t1;
      Tbar(1,0)=Tba_msg->r21;Tbar(1,1)=Tba_msg->r22;Tbar(1,2)=Tba_msg->r23;Tbar(1,3)=Tba_msg->t2;
      Tbar(2,0)=Tba_msg->r31;Tbar(2,1)=Tba_msg->r32;Tbar(2,2)=Tba_msg->r33;Tbar(2,3)=Tba_msg->t3;
      Tbar(3,0)=0;Tbar(3,1)=0;Tbar(3,2)=0;Tbar(3,3)=1;
    
      time_Tbar=ros::Time::now().toSec();
}

void trajectory_controller::limited_grasp_call(double theta_e,double theta4,double dx,double dy,int t_ex)
{
  static double last_call_time=0;

  if(ros::Time::now().toSec()-last_call_time>(double)t_ex/1000.0)
  {
    servoset_srv_msg.request.cmd=6;
    servoset_srv_msg.request.pos1=theta_e;
    servoset_srv_msg.request.pos2=theta4;//pi/2;
    servoset_srv_msg.request.pos3=dx;
    servoset_srv_msg.request.pos4=dy;
    servoset_srv_msg.request.action_time=t_ex;

    servoseter.call(servoset_srv_msg); 
    last_call_time=ros::Time::now().toSec();
  }
}

bool trajectory_controller::loop_valid_Tbar()
{
  if(ros::Time::now().toSec()-time_Tbar<0.1)return true;
  else return false;
}

bool trajectory_controller::loop_generate_trajectory()
{
   ROS_INFO("Loop in generate valid trajectory..");
   ros::Rate loop_rate(200);
   while(ros::ok())
   {
    if(loop_valid_Tbar())break;
    loop_rate.sleep();
    ros::spinOnce();
   }
   Tba=Tbar;
   ROS_INFO("Got valid Tba.");
   Matrix4d Tma=Tmb*Tba;
   double target_x,target_y,target_z;
   target_x=Tma(0,3);target_y=Tma(1,3);target_z=Tma(2,3);

   am_controller::trajectory_paraGoal current_task;
   current_task.theta_e= servoset_srv_msg.request.pos1;
   current_task.theta4= servoset_srv_msg.request.pos2;
   current_task.cx= servoset_srv_msg.request.pos3;
   current_task.cy= servoset_srv_msg.request.pos4;
   current_task.dx=target_x;
   current_task.dy=target_y;
   current_task.v0_0=0;current_task.v0_1=0;current_task.v0_2=0;
   ac.sendGoal(current_task);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(5));

    if (finished_before_timeout)
    {
      // actionlib::SimpleClientGoalState state = ac.getState();
      // ROS_INFO("opt trajectory generate done: %s",state.toString().c_str());
      am_controller::trajectory_paraResultConstPtr result_para=ac.getResult();
      if(result_para->t_a>0&&result_para->t_a<5.0)
      {
        #ifdef make_control_recorder
         target_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<< "start_target:"<<target_x<<" "<<target_y<<endl;
      #endif
        ROS_INFO("Generate trajectory succeed.");
        set_ab(result_para->para,result_para->t_a,result_para->t_b);
        return true;
      }
      else
      {
        ROS_INFO("Generate trajectory failed.");
        return false;
      }

      // Matrix4d mat;
      // m_trajectory.get_correctional_manipulator_pose(m_trajectory.t_a/2.0+m_trajectory.t_b/2.0,mat);

    }
    else 
    {
      ROS_INFO("Generate trajectory time out in one loop.");
      return false;
    }
}

bool trajectory_controller::get_target_control_para()
{
 ROS_INFO("Wait for trajectory opt server...");
 ac.waitForServer();
 ROS_INFO("Trajectory server found...");
 while(loop_generate_trajectory()==false);
 ROS_INFO("Trajectory generate done.");
 return true;
}
void trajectory_controller::init_manipulator()
{
      servoset_srv_msg.request.cmd=6;
      servoset_srv_msg.request.pos1=0;
      servoset_srv_msg.request.pos2=0;//pi/2;
      servoset_srv_msg.request.pos3=0.25;
      servoset_srv_msg.request.pos4=-0.00;
      servoset_srv_msg.request.action_time=3000;
      servoseter.call(servoset_srv_msg);
}
// void trajectory_controller::init_manipulator()
// {
//       servoset_srv_msg.request.cmd=6;
//       servoset_srv_msg.request.pos1=0;
//       servoset_srv_msg.request.pos2=0;//pi/2;
//       servoset_srv_msg.request.pos3=0;
//       servoset_srv_msg.request.pos4=-0.19;
//       servoset_srv_msg.request.action_time=3000;
//       servoseter.call(servoset_srv_msg);
// }

void trajectory_controller::trajectory_following()
{
  ROS_INFO("Loop trajectory  following....");
  Matrix4d Mat_SE3;
  int control_HZ=100;//10 
  int count_HZ=0;
  ros::Rate loop_rate(control_HZ);
  double start_time=ros::Time::now().toSec();

  #ifdef make_control_recorder
    correct_trajectory_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<< "start_time:"<<start_time<<endl;
  
  #endif

  while(ros::ok())
  {
    double pos1,pos2,pos3,pos4;
    double current_t=ros::Time::now().toSec()-start_time;
    get_correctional_manipulator_pose(current_t,Mat_SE3);
    // get_prepare_manipulator_pose(current_t,Mat_SE3);
    double te,t4,dx,dy;
    Kin.convert_SE3_tp(Mat_SE3,te,t4,dx,dy);
    t4=0;//some noisy may exist due to get_correctional_manipulator_pose.
    //printf("---\ntime:%lf\nte:%lf   t4:%lf   dx:%lf   dy:%lf\n",current_t,te,t4,dx,dy );
   #ifdef make_control_recorder
    correct_trajectory_file_recorder<<setprecision(9)<<setiosflags(ios::fixed)<<current_t<<" "<<dx<<" "<<dy<<endl;
  #endif
    // if(Kin.Check_valid(Mat_SE3,pos1,pos2,pos3,pos4))
    // {
    //   servoset_srv_msg.request.cmd=5;
    //   servoset_srv_msg.request.pos1=pos1;
    //   servoset_srv_msg.request.pos2=pos2;
    //   servoset_srv_msg.request.pos3=pos3;
    //   servoset_srv_msg.request.pos4=pos4;
    //   servoset_srv_msg.request.action_time=10;
    //   servoseter.call(servoset_srv_msg);
    // }
    count_HZ++;
    if(count_HZ>=10)
    {
      count_HZ=0;

                servoset_srv_msg.request.cmd=6;
              servoset_srv_msg.request.pos1=te;
              servoset_srv_msg.request.pos2=t4;
              servoset_srv_msg.request.pos3=dx;
              servoset_srv_msg.request.pos4=dy;
              servoset_srv_msg.request.action_time=400;//1000/control_HZ;
              servoseter.call(servoset_srv_msg);


              if(current_t>t_a+t_b+3.0)
              {
                static bool do_grasp=false;
                if(do_grasp==false)
                {
                    servoset_srv_msg.request.cmd=4;
                    servoset_srv_msg.request.action_time=2;
                    servoseter.call(servoset_srv_msg);
                    do_grasp=true;
                    ROS_INFO("start grasping!.");
                }
              }


              if(current_t>t_a+t_b+5.5)
              {
                ROS_INFO("trajectory follow done.");
                break;
              }
    }


    


    ros::spinOnce();
    loop_rate.sleep();
  }

}


void trajectory_controller::loop_publish_task_success(void)
{
  am_controller::success_flag flag_msg;
  flag_msg.success_flag=1;
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
     trajectory_success_flag_puber.publish(flag_msg);
     ros::spinOnce();
     loop_rate.sleep();
  }
}

void trajectory_controller::wait_for_task()
{
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
     if(task_rev==true)break;
     ros::spinOnce();
     loop_rate.sleep();
  }
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_controller");
  trajectory_controller m_trajectory;
/*
 /* #ifdef make_control_recorder
    target_file_recorder.open("/home/flx/catkin_al/target_recorder.txt",ios::out|ios::trunc);
    correct_trajectory_file_recorder.open("/home/flx/catkin_al/correct_trajectory_recorder.txt",ios::out|ios::trunc);
    if(target_file_recorder==NULL)ROS_INFO("NULL FILE PTR1");   
    if(correct_trajectory_file_recorder==NULL)ROS_INFO("NULL FILE PTR2");   
  #endif*/

  m_trajectory.wait_for_task();
  m_trajectory.init_manipulator();
  ros::Duration(3.0).sleep();//wait for manipulator initial.
  m_trajectory.get_target_control_para();
  m_trajectory.trajectory_following();
  m_trajectory.loop_publish_task_success();

#ifdef make_control_recorder
    target_file_recorder.close();
    correct_trajectory_file_recorder.close();
  #endif

  return 0;
}
