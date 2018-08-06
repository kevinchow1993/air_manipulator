#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


def cur_pose_cb(msg):
    global cur_pose
    rospy.loginfo("cur pose: x: %.4f y: %.4f"%(msg.pose.position.x, msg.pose.position.y))
    cur_pose.header.stamp = rospy.Time.now()
    cur_pose.pose.position.x = msg.pose.position.x
    cur_pose.pose.position.y = msg.pose.position.y
    cur_pose.pose.position.z = msg.pose.position.z

def set_pose_cb(msg):
    global set_pose
    rospy.loginfo("set pose: x: %.4f y: %.4f"%(msg.pose.position.x, msg.pose.position.y))
    set_pose.header.stamp = rospy.Time.now()
    set_pose.pose.position.x = msg.pose.position.x
    set_pose.pose.position.y = msg.pose.position.y
    set_pose.pose.position.z = msg.pose.position.z

if __name__ == "__main__":
    rospy.init_node("test_subscriber")
    rospy.loginfo("Start to subscribe")
    sub_cur_pose = rospy.Subscriber("/vrpn/uav_pose", PoseStamped, cur_pose_cb)
    sub_set_pose = rospy.Subscriber("/PlanningUav/uav_set_pose", PoseStamped, set_pose_cb)

    cur_pose = PoseStamped()
    set_pose = PoseStamped()

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        diff_x = set_pose.pose.position.x - cur_pose.pose.position.x
        diff_y = set_pose.pose.position.y - cur_pose.pose.position.y
        diff_z = set_pose.pose.position.z - cur_pose.pose.position.z

        br.sendTransform((diff_x, diff_y, diff_z), (0., 0., 0., 1.), rospy.Time.now(), "uav", "world")
