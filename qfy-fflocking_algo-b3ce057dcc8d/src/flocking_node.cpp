//#include "fflocking/flocking_algo.hpp"
#include "fflocking/planning_ros.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "Flocking_node");
  ros::NodeHandle n("~");
  FlockingRos::PlanningUav planning_uav;

  //initialize Height and Orientation

  //Later this part should be modified as dynamic config.
  //-----------------------------------------------------//
  planning_uav.setUavPoseZ(0.8);
  planning_uav.setUavPoseOrie(0., 0., 0.);
  //-----------------------------------------------------//

  ros::Rate r(100);
  while(ros::ok())
  {
    if(planning_uav.checkCallbackReady()){
      planning_uav.setUavPoseXY();
      planning_uav.publishUavSetPose();
    }

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
