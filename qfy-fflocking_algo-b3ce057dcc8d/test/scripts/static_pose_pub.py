#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('static_pose_pub', anonymous=True)
    target_pub = rospy.Publisher('/vrpn/target_pose', PoseStamped, queue_size=10)
    obstacle_pub = rospy.Publisher('/vrpn/obstacle_pose', PoseStamped, queue_size=10)
    uav_pose_pub = rospy.Publisher('/vrpn/uav_pose', PoseStamped, queue_size=10)
    uav_vel_pub = rospy.Publisher('/vrpn/uav_vel', PoseStamped, queue_size=10)

    target_pose = PoseStamped()
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 1.0
    target_pose.pose.position.z = 0.2
    target_pose.pose.orientation.x = 0.
    target_pose.pose.orientation.y = 0.
    target_pose.pose.orientation.z = 0.
    target_pose.pose.orientation.w = 1.

    obstacle_pose = PoseStamped()
    obstacle_pose.pose.position.x = 0.5
    obstacle_pose.pose.position.y = 0.5
    obstacle_pose.pose.position.z = 0.4
    obstacle_pose.pose.orientation.x = 0.
    obstacle_pose.pose.orientation.y = 0.
    obstacle_pose.pose.orientation.z = 0.
    obstacle_pose.pose.orientation.w = 1.

    uav_pose = PoseStamped()
    uav_pose.pose.position.x = 0.0
    uav_pose.pose.position.y = 0.0
    uav_pose.pose.position.z = 0.8
    uav_pose.pose.orientation.x = 0.
    uav_pose.pose.orientation.y = 0.
    uav_pose.pose.orientation.z = 0.
    uav_pose.pose.orientation.w = 1.

    uav_vel = PoseStamped()
    uav_pose.pose.position.x = 0.0
    uav_pose.pose.position.y = 0.0
    uav_pose.pose.position.z = 0.0
    uav_pose.pose.orientation.x = 0.
    uav_pose.pose.orientation.y = 0.
    uav_pose.pose.orientation.z = 0.
    uav_pose.pose.orientation.w = 1.

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        obstacle_pose.header.stamp = rospy.Time.now()
        uav_pose.header.stamp = rospy.Time.now()
        uav_vel.header.stamp = rospy.Time.now()

        target_pub.publish(target_pose)
        obstacle_pub.publish(obstacle_pose)
        uav_pose_pub.publish(uav_pose)
        uav_vel_pub.publish(uav_vel)

        rate.sleep()



