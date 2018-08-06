#ifndef PLANNING_ROS_H
#define PLANNING_ROS_H

#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include "geometry_msgs/PoseStamped.h"
#include "fflocking/flocking_algo.hpp"
#include "fflocking/FlockingAlgoConfig.h"
#include "dynamic_reconfigure/server.h"
#include "visualization_msgs/Marker.h"
#include <vector>

//using namespace Eigen;

namespace FlockingRos
{
class PlanningUav{
public:
  PlanningUav();
  ~PlanningUav(){}
  bool checkCallbackReady();
  void setUavPoseXY();
  void setUavPoseZ(const double&);
  void setUavPoseOrie(const tf::Quaternion&);
  void setUavPoseOrie(const double & , const double &, const double &);//roll, pitch, yaw
  void publishUavSetPose();

private:
  double uav_height_;
  double step_const_;
  tf::Quaternion uav_quat_;

  ros::NodeHandle n_;
  ros::Subscriber target_sub_;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber uav_pose_sub_;
  ros::Subscriber uav_vel_sub_;

  ros::Publisher uav_set_pos_pub_;
  ros::Publisher uav_set_visualize_arrow_pub_;

  geometry_msgs::PoseStamped target_pose_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped uav_pose_;
  geometry_msgs::PoseStamped uav_vel_;
  geometry_msgs::PoseStamped uav_set_pose_;
  tf::TransformBroadcaster tf_br_;


  bool isTarget_, isObstacle_, isUavPose_, isUavVel_;

  void target_cb_(const geometry_msgs::PoseStamped& );
  void obstacle_cb_(const geometry_msgs::PoseStamped& );
  void uav_pose_cb_(const geometry_msgs::PoseStamped& );
  void uav_vel_cb_(const geometry_msgs::PoseStamped& );
  void init_pose();
  void dyna_para_cb_(fflocking::FlockingAlgoConfig& , uint32_t);
  void update_para_(fflocking::FlockingAlgoConfig&);

  FlockingAlgo flocking_;

  Eigen::Vector2f vec_uav_pose_,
           vec_uav_vel_,
           vec_target_pose_,
           vec_target_vel_,
           vec_obstacle_pose_,
           vec_obstacle_vel_;

  Eigen::Vector2f vec_set_pose_;

  dynamic_reconfigure::Server<fflocking::FlockingAlgoConfig> dyna_server_;
  dynamic_reconfigure::Server<fflocking::FlockingAlgoConfig>::CallbackType bind_func_;
  visualization_msgs::Marker vis_arrow_;
  std::vector<geometry_msgs::Point>::iterator it_arrow_point_;
  tf::Transform transform_;

  geometry_msgs::Point pre_uav_point_;
  geometry_msgs::Point cur_uav_point_;
  ros::Time timer_,pre_timer_;

};

}
#endif
