#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

server = None
br = None
counter = 0

target_pose = Point()
obstacle_pose = Point()
uav_pose = Point()


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    # br.sendTransform((0, 0, sin(counter / 140.0) * 2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame")
    # br.sendTransform((0, 0, sin(counter / 140.0) * 2.0), (0, 0, 0, 1.0), time, "base_link", "target")
    # br.sendTransform((0, 0, sin(counter / 140.0) * 2.0), (0, 0, 0, 1.0), time, "base_link", "obstacle")
    # br.sendTransform((0, 0, sin(counter / 140.0) * 2.0), (0, 0, 0, 1.0), time, "base_link", "uav")
    counter += 1


def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
    server.applyChanges()

def makeBox(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 1.
    marker.color.g = 0.
    marker.color.b = 0.
    marker.color.a = 1.

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


# def saveMarker(int_marker):
#     server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def make6DofMarker(name, fixed, interaction_mode, position, show_6dof=False):
    int_marker = InteractiveMarker()
    int_marker.header.stamp = rospy.Time.now()
    int_marker.header.frame_id = "base_link"
    # int_marker.child_frame_id = name
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = name
    int_marker.description = name

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)


if __name__ == "__main__":
    rospy.init_node("basic_controls")

    br = TransformBroadcaster()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")


    name1 = "target"
    position = Point(0, 0, 0)
    make6DofMarker(name1, False, InteractiveMarkerControl.MOVE_3D, position, True)
    
    name2 = "obstacle"
    position = Point(1., 1., 0.2)
    make6DofMarker(name2, False, InteractiveMarkerControl.MOVE_3D, position, True)

    name3 = "uav"
    position = Point(0.5, 0.5, 0.5)
    make6DofMarker(name3, False, InteractiveMarkerControl.MOVE_3D, position, True)

    # position = Point(0, -6, 0)
    # makeMovingMarker(position)

    server.applyChanges()

    rospy.spin()

