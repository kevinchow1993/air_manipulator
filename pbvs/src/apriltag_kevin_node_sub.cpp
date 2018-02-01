#include "pbvs/apriltag_info.h"
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_kevin_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  apriltags_ros::AprilTagDetector detector(nh, pnh);
  ros::spin();
}
