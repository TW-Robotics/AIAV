#!/usr/bin/env python3

# Dieses Skript generiert ein simuliertes Objekt, welches der Roboter aufheben kann.
# Die Implementierung basiert auf dem Interactive Marker Tutorial in der ROS Wiki.
# Tutorial: http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import tf2_ros
import geometry_msgs.msg

""" Creates an interactive RVIZ marker that publishes a transform between object_tf and world coordinate frames based on the current marker position. """

def processFeedback(feedback):
    br = tf2_ros.TransformBroadcaster()
    pt = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(pt.x) + ", " + str(pt.y) + ", " + str(pt.z))
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "object_tf"
    t.transform.translation.x = pt.x
    t.transform.translation.y = pt.y
    t.transform.translation.z = pt.z
    q = [0,0,0,1]
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)


if __name__=="__main__":
    rospy.init_node("tcp_marker")
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.name = "my_marker"
    int_marker.description = "Fake Object Control"

    ######   BOX MARKER    ######
    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.02
    box_marker.scale.y = 0.02
    box_marker.scale.z = 0.02
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )
    box_control.orientation.w = 1
    box_control.orientation.x = 0
    box_control.orientation.y = 1
    box_control.orientation.z = 0
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    #######  MOVE IN  XY PLANE MARKER  #######
    # remove comment in order to create dedicated arrows to move the marker along the xy-plane
    #plane_control = InteractiveMarkerControl()
    #plane_control.orientation.w = 1
    #plane_control.orientation.x = 0
    #plane_control.orientation.y = 1
    #plane_control.orientation.z = 0
    #plane_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    #plane_control.name = "move_xy_plane"
    #int_marker.controls.append(plane_control)


    #########    MOVE ALONG Z AXIS    #########
    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    z_control = InteractiveMarkerControl()
    z_control.orientation.w = 1
    z_control.orientation.x = 0
    z_control.orientation.y = 1
    z_control.orientation.z = 0
    z_control.name = "move_z"
    z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(z_control);

    # set marker scale
    int_marker.scale = 0.1

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()