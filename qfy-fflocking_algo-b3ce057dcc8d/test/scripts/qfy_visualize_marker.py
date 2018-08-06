#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, PoseStamped
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

class QFYMarker(object):
    def __init__(self, name, fixed, position, show6dof):
        self.server_ = InteractiveMarkerServer("QFY_Marker"+ name)
        self.br = TransformBroadcaster()
        self.marker_ = Marker()
        self.control_ = InteractiveMarkerControl()
        self.inter_marker_ = InteractiveMarker()
        self.position_ = position
        self.name_ = name
        self.fixed_ = fixed

        self.pub_pose = rospy.Publisher('/vrpn/'+self.name_+'_pose', PoseStamped, queue_size=10)

        self.init_time = rospy.Time(0)

        if self.name_ is "uav":
            self.step_const = 0.01
            self.uav_set_pose = PoseStamped()

            self.uav_trajectory_line = rospy.Publisher("visulization_uav_trajectory", Marker, queue_size=10)
            self.traj_line_strip = Marker()
            self.traj_line_strip.header.frame_id = "world"
            self.traj_line_strip.header.stamp = rospy.Time.now()
            self.traj_line_strip.ns = "uav_trajectory"
            self.traj_line_strip.action = Marker.ADD

            self.traj_line_strip.pose.orientation.w = 1.0
            self.traj_line_strip.id = 0
            self.traj_line_strip.type = Marker.LINE_STRIP

            self.traj_line_strip.scale.x = 0.01
            self.traj_line_strip.scale.y = 0.01

            self.traj_line_strip.color.a = 1.0
            self.traj_line_strip.color.g = 1.0
            self.traj_line_strip.color.r = 1.0


            self.uav_init_pose = PoseStamped()
            self.uav_init_pose.pose.position.x = position.x
            self.uav_init_pose.pose.position.y = position.y
            self.uav_init_pose.pose.position.z = position.z

            self.uav_cur_pose = PoseStamped()
            self.uav_cur_pose.pose.position.x = self.uav_init_pose.pose.position.x
            self.uav_cur_pose.pose.position.y = self.uav_init_pose.pose.position.y
            self.uav_cur_pose.pose.position.z = self.uav_init_pose.pose.position.z

            self.sub_pose = rospy.Subscriber('/PlanningUav/uav_set_pose', PoseStamped, self.sub_uav_set_cb)
            self.timer_cb = rospy.Timer(rospy.Duration(0.03), self.set_uav_pose)

        self.make6DofMarker(self.name_, self.fixed_, self.position_)

    def set_uav_pose(self, msg):
        self.uav_cur_pose.pose.position.x = self.uav_set_pose.pose.position.x
        self.uav_cur_pose.pose.position.y = self.uav_set_pose.pose.position.y
        self.uav_cur_pose.pose.position.z = self.uav_set_pose.pose.position.z

        self.traj_line_strip.points.append(copy.deepcopy(self.uav_set_pose.pose.position))
        self.uav_trajectory_line.publish(self.traj_line_strip)

    def sub_uav_set_cb(self, msg):
        self.uav_set_pose.pose.position.x = self.step_const * msg.pose.position.x + \
                                            (1 - self.step_const) * self.uav_cur_pose.pose.position.x
        self.uav_set_pose.pose.position.y = self.step_const * msg.pose.position.y + \
                                            (1 - self.step_const) * self.uav_cur_pose.pose.position.y
        self.uav_set_pose.pose.position.z = self.step_const * msg.pose.position.z + \
                                            (1 - self.step_const) * self.uav_cur_pose.pose.position.z

    def pubPose(self):
        if self.name_ is not "uav":
            self.br.sendTransform((self.inter_marker_.pose.position.x, self.inter_marker_.pose.position.y, self.inter_marker_.pose.position.z),
                              (0., 0., 0., 1.),rospy.Time(0), self.name_, "world")
        else:
            self.inter_marker_.pose.position.x = self.uav_cur_pose.pose.position.x
            self.inter_marker_.pose.position.y = self.uav_cur_pose.pose.position.y
            self.inter_marker_.pose.position.z = self.uav_cur_pose.pose.position.z
            self.server_.applyChanges()
            self.br.sendTransform((self.uav_cur_pose.pose.position.x, self.uav_cur_pose.pose.position.y,
                                   self.uav_cur_pose.pose.position.z),
                                  (0., 0., 0., 1.), rospy.Time(0), "uav", "world")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        if self.name_ is not "uav":
            pose.pose.position.x = self.inter_marker_.pose.position.x
            pose.pose.position.y = self.inter_marker_.pose.position.y
            pose.pose.position.z = self.inter_marker_.pose.position.z
        else:
            pose.pose.position.x = self.uav_cur_pose.pose.position.x
            pose.pose.position.y = self.uav_cur_pose.pose.position.y
            pose.pose.position.z = self.uav_cur_pose.pose.position.z
        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = 0.
        pose.pose.orientation.w = 1.
        self.pub_pose.publish(pose)

    def makeBox(self, msg):
        if self.name_ is 'obstacle':
            self.marker_.type = Marker.CYLINDER
            self.marker_.scale.x = msg.scale * 1.25
            self.marker_.scale.y = msg.scale * 1.25
            self.marker_.scale.z = msg.scale * 1.25
            self.marker_.color.r = 1.
            self.marker_.color.g = 0.
            self.marker_.color.b = 0.
            self.marker_.color.a = 0.7
        if self.name_ is 'target':
            self.marker_.type = Marker.CYLINDER
            self.marker_.scale.x = 0.75
            self.marker_.scale.y = 0.75
            self.marker_.scale.z = 5.
            self.marker_.color.r = 1.
            self.marker_.color.g = 0.
            self.marker_.color.b = 0.
            self.marker_.color.a = 0.5
        if self.name_ is 'uav':
            self.marker_.type = Marker.CYLINDER
            self.marker_.scale.x = 0.6
            self.marker_.scale.y = 0.6
            self.marker_.scale.z = 0.2
            self.marker_.color.r = 0.
            self.marker_.color.g = 0.
            self.marker_.color.b = 1.
            self.marker_.color.a = 0.5

        return self.marker_

    def makeBoxControl(self, msg):
        self.control_ = InteractiveMarkerControl()
        self.control_.always_visible = True
        self.control_.markers.append(self.makeBox(msg))
        msg.controls.append(self.control_)
        return self.control_

    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x ** 2 + quaternion_msg.y ** 2 + quaternion_msg.z ** 2 + quaternion_msg.w ** 2
        s = norm ** (-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")

        self.server_.applyChanges()

    def make6DofMarker(self, name, fixed, position, show_6dof=False):
        self.inter_marker_.header.stamp = rospy.Time(0)
        self.inter_marker_.header.frame_id = "world"
        # if self.name_ is not 'uav':
        #     self.inter_marker_.header.frame_id = "world"
        # else:
        #     self.inter_marker_.header.frame_id = "uav_movingframe"

        self.inter_marker_.pose.position = position
        self.inter_marker_.scale = 0.5

        self.inter_marker_.name = name
        self.inter_marker_.description = name

        self.makeBoxControl(self.inter_marker_)
        self.inter_marker_.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

        if show_6dof:
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            self.normalizeQuaternion(control.orientation)
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            self.normalizeQuaternion(control.orientation)
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            self.normalizeQuaternion(control.orientation)
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            self.normalizeQuaternion(control.orientation)
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            self.normalizeQuaternion(control.orientation)
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            self.normalizeQuaternion(control.orientation)
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            self.inter_marker_.controls.append(control)

        self.pubPose()
        self.server_.insert(self.inter_marker_, self.processFeedback)


if __name__ == "__main__":
    rospy.init_node("basic_controls")

    position = Point(1, 0, 0)
    target = QFYMarker('target', False, position, True)
    target.server_.applyChanges()

    position = Point(0.2, 0.2, 0.6)
    obstacle = QFYMarker('obstacle', False, position, True)
    obstacle.server_.applyChanges()

    position = Point(-1.0, 1.5, 0.)
    uav = QFYMarker('uav', False, position, True)
    uav.server_.applyChanges()
    # uav.pubPose()

    rospy.sleep(rospy.Duration(2))
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        target.pubPose()
        obstacle.pubPose()
        uav.pubPose()

        rate.sleep()