#! /usr/bin/env  python
#! coding=utf-8

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import math

if __name__ == '__main__':
    rospy.init_node('tango_pose_broadcaster')

    listener = tf.TransformListener()
    broadcaster1 = tf.TransformBroadcaster()
    broadcaster2 = tf.TransformBroadcaster()

    tango_pose =  rospy.Publisher('tango_pose',PoseStamped,queue_size=10)

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        broadcaster1.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, -math.pi / 2),
                                   rospy.Time.now(),
                                   "/start_of_service", '/origin')
        broadcaster2.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0,math.pi/2,math.pi/2),rospy.Time.now(),
                        "/tangoNED",'/device')
        try:
            (trans, rot) = listener.lookupTransform('/origin', '/tangoNED', rospy.Time(0))
        except:
            rospy.logwarn("transform exception")
            continue

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "origin"
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]

        # quaternion = tf.transformations.quaternion_from_euler(imu_roll,imu_pitch,theta)
        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]

        tango_pose.publish(msg)

        rate.sleep()