#ifndef TRAJECTORY_CONTROLLER_H
#define TRAJECTORY_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <am_controller/trajectory_paraAction.h>
#include <am_controller/Mat_Tba.h>
#include <vector>
#include <Eigen/Eigen> 
#include "Kin.h"
#include <am_controller/servoset_srv.h>
#include <am_controller/success_flag.h>
#include <camera_control/do_trajectory_task.h>

using namespace std;
using namespace Eigen; 


class trajectory_controller
{
  struct para_ab
  {
    double a[6],b[6];
  };
  public:
  ros::NodeHandle n;
  ros::Subscriber Tba_suber;
  ros::Subscriber Task_suber;
  ros::ServiceClient servoseter;
  ros::Publisher trajectory_success_flag_puber;
  actionlib::SimpleActionClient<am_controller::trajectory_paraAction> ac;
  FLX_kinematics Kin;
  Matrix4d Tba,Tbar,Tmb;
  am_controller::servoset_srv servoset_srv_msg;
  double time_Tbar;
  bool task_rev;

  
  para_ab link[3];
  double t_a,t_b;
  trajectory_controller();
  ~trajectory_controller();
  void init_Tmb();
  void set_ab(const vector<double> &para,const double &t_a_,const double &t_b_);
  Matrix4d get_inverse_SE3(Matrix4d M12);
  double get_link_angle_a(int link_no,double t);
  double get_link_angle_b(int link_no,double t);
  double get_link_vel_a(int link_no,double t);
  double get_link_vel_b(int link_no,double t);
  double get_link_acc_a(int link_no,double t);
  double get_link_acc_b(int link_no,double t);
  double get_link_angle(int link_no,double t);
  void get_prepare_manipulator_pose(double t,Matrix4d &result_SE3);
  void get_correctional_manipulator_pose(double t,Matrix4d &result_SE3);
  void Tba_callback(const am_controller::Mat_Tba::ConstPtr &Tba_msg);
  bool get_target_control_para();
  bool loop_generate_trajectory();
  bool loop_valid_Tbar();
  void limited_grasp_call(double theta_e,double theta4,double dx,double dy,int t_ex);
  void init_manipulator();
  void trajectory_following();
  void loop_publish_task_success();
  void wait_for_task();
  void Task_callback(const camera_control::do_trajectory_taskConstPtr &msg);
};




#endif