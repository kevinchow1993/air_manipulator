#include "fflocking/planning_ros.hpp"

using namespace FlockingRos;

PlanningUav::PlanningUav():
  n_("PlanningUav"),
  isTarget_(false),
  isObstacle_(false),
  isUavPose_(false),
  isUavVel_(true),
  uav_height_(0.8),
  uav_quat_(0., 0., 0., 1.),
  step_const_(0.01)
{
  target_sub_ = n_.subscribe("/vrpn/target_pose", 10, &PlanningUav::target_cb_, this);
  obstacle_sub_ = n_.subscribe("/vrpn/obstacle_pose", 10, &PlanningUav::obstacle_cb_, this);
  uav_pose_sub_ = n_.subscribe("/mavros/vision_pose", 10, &PlanningUav::uav_pose_cb_, this);
//  uav_vel_sub_ = n_.subscribe("/vrpn/uav_vel", 10, &PlanningUav::uav_vel_cb_, this);

  uav_set_pos_pub_ = n_.advertise<geometry_msgs::PoseStamped>("uav_set_pose",100);
  uav_set_visualize_arrow_pub_ = n_.advertise<visualization_msgs::Marker>("uav_visualization", 100);

  bind_func_ = boost::bind(&PlanningUav::dyna_para_cb_, this, _1, _2);
  dyna_server_.setCallback(bind_func_);
  init_pose();
}

void PlanningUav::init_pose()
{//intialize pose variables
 //Attention: use PoseStamped msg to substitute velocity info of uav_vel
  target_pose_.header.stamp = ros::Time::now();
  target_pose_.pose.position.x = 0.;
  target_pose_.pose.position.y = 0.;
  target_pose_.pose.position.z = 0.;
  target_pose_.pose.orientation.x = 0.;
  target_pose_.pose.orientation.y = 0.;
  target_pose_.pose.orientation.z = 0.;
  target_pose_.pose.orientation.w = 1.;

  obstacle_pose_.header.stamp = ros::Time::now();
  obstacle_pose_.pose.position.x = 0.;
  obstacle_pose_.pose.position.y = 0.;
  obstacle_pose_.pose.position.z = 0.;
  obstacle_pose_.pose.orientation.x = 0.;
  obstacle_pose_.pose.orientation.y = 0.;
  obstacle_pose_.pose.orientation.z = 0.;
  obstacle_pose_.pose.orientation.w = 1.;

  uav_pose_.header.stamp = ros::Time::now();
  uav_pose_.pose.position.x = 0.;
  uav_pose_.pose.position.y = 0.;
  uav_pose_.pose.position.z = 0.;
  uav_pose_.pose.orientation.x = 0.;
  uav_pose_.pose.orientation.y = 0.;
  uav_pose_.pose.orientation.z = 0.;
  uav_pose_.pose.orientation.w = 1.;

  uav_vel_.header.stamp = ros::Time::now();
  uav_vel_.pose.position.x = 0.;
  uav_vel_.pose.position.y = 0.;
  uav_vel_.pose.position.z = 0.;
  uav_vel_.pose.orientation.x = 0.;
  uav_vel_.pose.orientation.y = 0.;
  uav_vel_.pose.orientation.z = 0.;
  uav_vel_.pose.orientation.w = 1.;

  vec_uav_pose_ << 0., 0.;
  vec_uav_vel_ << 0., 0.;
  vec_target_pose_ << 0., 0.;
  vec_target_vel_ << 0., 0.;
  vec_obstacle_pose_ << 0., 0.;
  vec_obstacle_vel_ << 0., 0.;

  vec_set_pose_ << 0., 0.;

  vis_arrow_.header.frame_id = "uav";
  vis_arrow_.ns = "planning_uav";
  vis_arrow_.id = 0;
  vis_arrow_.type = visualization_msgs::Marker::ARROW;
//  vis_arrow_.action = visualization_msgs::Marker::ADD;
  vis_arrow_.scale.x = 0.05;
  vis_arrow_.scale.y = 0.1;
  vis_arrow_.scale.z = 0.1;
  vis_arrow_.color.a = 1.0;
  vis_arrow_.color.r = 0.0;
  vis_arrow_.color.g = 1.0;
  vis_arrow_.color.b = 0.0;
//  vis_arrow_.frame_locked = true;

  pre_timer_ = ros::Time::now();
  timer_ = ros::Time::now();
}
// update target position
void PlanningUav::target_cb_(const geometry_msgs::PoseStamped& msg_pose_stamped){
  isTarget_ = true;
  target_pose_.header.stamp = msg_pose_stamped.header.stamp;
  target_pose_.pose.position.x = msg_pose_stamped.pose.position.x;
  target_pose_.pose.position.y = msg_pose_stamped.pose.position.y;
}

//update obstacle position
void PlanningUav::obstacle_cb_(const geometry_msgs::PoseStamped& msg_pose_stamped)
{
  isObstacle_ = true;
  obstacle_pose_.header.stamp = msg_pose_stamped.header.stamp;
  obstacle_pose_.pose.position.x = msg_pose_stamped.pose.position.x;
  obstacle_pose_.pose.position.y = msg_pose_stamped.pose.position.y;
}

//update the uav velocity and position
void PlanningUav::uav_pose_cb_(const geometry_msgs::PoseStamped& msg_pose_stamped)
{
  isUavPose_ = true;
  timer_ = ros::Time::now();
  double period = timer_.toSec() - pre_timer_.toSec();

  uav_pose_.header.stamp = msg_pose_stamped.header.stamp;
  uav_pose_.pose.position.x = msg_pose_stamped.pose.position.x;
  uav_pose_.pose.position.y = msg_pose_stamped.pose.position.y;
  uav_pose_.pose.position.z = msg_pose_stamped.pose.position.z;

  cur_uav_point_.x = msg_pose_stamped.pose.position.x;
  cur_uav_point_.y = msg_pose_stamped.pose.position.y;

  uav_vel_.header.stamp = msg_pose_stamped.header.stamp;
  uav_vel_.pose.position.x = (cur_uav_point_.x - pre_uav_point_.x) / period;
  uav_vel_.pose.position.y = (cur_uav_point_.y - pre_uav_point_.y) / period;

  pre_uav_point_.x = msg_pose_stamped.pose.position.x;
  pre_uav_point_.y = msg_pose_stamped.pose.position.y;
  ROS_INFO("\nuav_vel x : %.4f, y : %.4f",
           uav_vel_.pose.position.x,
           uav_vel_.pose.position.y);
}

//void PlanningUav::uav_vel_cb_(const geometry_msgs::PoseStamped& msg_pose_stamped)
//{
//  isUavVel_ = true;
//  uav_vel_.header.stamp = msg_pose_stamped.header.stamp;
//  uav_vel_.pose.position.x = msg_pose_stamped.pose.position.x;
//  uav_vel_.pose.position.y = msg_pose_stamped.pose.position.y;
//}
//动态调参
void PlanningUav::dyna_para_cb_(fflocking::FlockingAlgoConfig& config, uint32_t level){
  ROS_INFO("Reconfigure Request: \n"
           "  alpha:  %f\n"
           "  gama:   %f\n"
           "  betai:  %f\n"
           "  k1:     %f\n"
           "  kl:     %f\n"
           "  kij:    %f\n"
           "  kp:     %f\n"
           "  T:      %f\n"
           "  Mi:     %f\n"
           "  d:      %f\n"
           "  default_radius:   %f\n"
           "  obstacle_radius:  %f\n"
           "  satu_thres:       %f\n"
           "  m_radius:         %f\n"
           "  m_ringWidth:      %f\n",
           config.alpha,
           config.gama,
           config.betai,
           config.k1,
           config.kl,
           config.kij,
           config.kp,
           config.T,
           config.Mi,
           config.d,
           config.default_radius,
           config.obstacle_radius,
           config.satu_thres,
           config.m_radius,
           config.m_ringWidth);
  update_para_(config);
  if(checkCallbackReady()){
    setUavPoseXY();
    ROS_INFO("Recompute Set XY!!");
  }
}

//更新参数
void PlanningUav::update_para_(fflocking::FlockingAlgoConfig& config){
  flocking_.updatePara(config.alpha, config.gama,
                       config.Mi,    config.betai,
                       config.k1,    config.kl,
                       config.kij,   config.obstacle_radius,
                       config.kp,    config.T,
                       config.d,     config.default_radius,
                       config.satu_thres,  config.m_radius,
                       config.m_ringWidth);
}

//判断是否进入回调函数
bool PlanningUav::checkCallbackReady(){
//  if(isTarget_ && isObstacle_ && isUavPose_ && isUavVel_)
//    ROS_INFO("HAHA");
  return (isTarget_ && isObstacle_ && isUavPose_ && isUavVel_);
}

//将目标物，障碍物，和飞机位置传入 算法
void PlanningUav::setUavPoseXY()
{//Attention: in the version target_vel_ and obstacle_vel set to zero
  vec_target_pose_ << target_pose_.pose.position.x, target_pose_.pose.position.y;
  vec_obstacle_pose_ << obstacle_pose_.pose.position.x, obstacle_pose_.pose.position.y;
  vec_uav_pose_ << uav_pose_.pose.position.x, uav_pose_.pose.position.y;
  vec_uav_vel_ << uav_vel_.pose.position.x, uav_vel_.pose.position.y;

  //计算无人机下一时刻
  vec_set_pose_ = flocking_.calInputKinemicRing(vec_uav_pose_,
                                vec_uav_vel_,
                                vec_target_pose_,
                                vec_target_vel_,
                                vec_obstacle_pose_,
                                vec_obstacle_vel_);
}

//设置飞机飞行高度
void PlanningUav::setUavPoseZ(const double& z){
  uav_height_ = z;
}

//设置无人机方向
void PlanningUav::setUavPoseOrie(const tf::Quaternion& q){
  uav_quat_ = q;
}

//函数重载，设置无人机方向
void PlanningUav::setUavPoseOrie(const double & roll,
                                 const double & pitch,
                                 const double & yaw){
  uav_quat_ = tf::createQuaternionFromRPY(roll, pitch, yaw);
}

//将设置点发布出来
void PlanningUav::publishUavSetPose(){
  //update uav_set_pose_

  //for rviz
  uav_set_pose_.pose.position.x = vec_set_pose_(0);
  uav_set_pose_.pose.position.y = vec_set_pose_(1);

  uav_set_pose_.pose.position.z = uav_height_;
  uav_set_pose_.pose.orientation.x = uav_quat_.x();
  uav_set_pose_.pose.orientation.y = uav_quat_.y();
  uav_set_pose_.pose.orientation.z = uav_quat_.z();
  uav_set_pose_.pose.orientation.w = uav_quat_.w();

  //for rviz
//  geometry_msgs::Point begin_point, end_point;

//  begin_point.x = 0.;
//  begin_point.y = 0.;
//  begin_point.z = 0.;

//  end_point.x = uav_set_pose_.pose.position.x - uav_pose_.pose.position.x;
//  end_point.y = uav_set_pose_.pose.position.y - uav_pose_.pose.position.y;
//  end_point.z = uav_set_pose_.pose.position.z - uav_pose_.pose.position.z;

//  vis_arrow_.points.erase(vis_arrow_.points.begin(), vis_arrow_.points.end());
//  vis_arrow_.points.push_back(begin_point);
//  vis_arrow_.points.push_back(end_point);
//  vis_arrow_.points.resize(2);
//  uav_set_visualize_arrow_pub_.publish(vis_arrow_);
//发布tf
  transform_.setOrigin(tf::Vector3(uav_set_pose_.pose.position.x,
                                   uav_set_pose_.pose.position.y,
                                   uav_set_pose_.pose.position.z));
  tf::Quaternion q(uav_set_pose_.pose.orientation.x,
                   uav_set_pose_.pose.orientation.y,
                   uav_set_pose_.pose.orientation.z,
                   uav_set_pose_.pose.orientation.w);
  transform_.setRotation(q);


  tf_br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),
                                            "world", "uav_set_pose"));
  uav_set_pos_pub_.publish(uav_set_pose_);

}
