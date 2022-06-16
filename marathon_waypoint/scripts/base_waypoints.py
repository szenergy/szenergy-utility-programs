#!/usr/bin/env python

from matplotlib import transforms
import rospy
from std_msgs.msg import String
from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import math
import tf2
import tf2_geometry_msgs


current_pose = None
pub = None
marker_pub = None

tf_buffer = None

def current_pose_callback(msg):
    global current_pose
    current_pose = msg

def waypoints_callback(waypoints_msg):
    global current_pose,pub,marker_pub,tf_buffer

    euc_dist = 1000000
    euc_index = 0

    transform = tf_buffer.lookup_transform("base_link","map")


    for i in range(0,len(waypoints_msg.waypoints)):
        current_dist = math.sqrt(pow(current_pose.pose.position.x - waypoints_msg.waypoints[i].pose.pose.position.x,2) + pow(current_pose.pose.position.y - waypoints_msg.waypoints[i].pose.pose.position.y,2))

        if current_dist < euc_dist:
            euc_dist = current_dist
            euc_index = i

    lane_sender = Lane()
    line_strip = Marker()
    for waypoint in lane_sender.waypoints:
        transformed_pose = tf2_geometry_msgs.do_transform(waypoint.pose,transform)
        lane_sender.waypoints.append(transformed_pose)
        


    lane_sender.waypoints = waypoints_msg.waypoints[euc_index:euc_index+99]
    lane_sender.header = waypoints_msg.header
    for waypoint in lane_sender.waypoints:
        point = Point()
        point.x = waypoint.pose.pose.position.x
        point.y = waypoint.pose.pose.position.y
        point.z = waypoint.pose.pose.position.z
        line_strip.points.append(point)

    
    line_strip.header = lane_sender.header
    line_strip.type = Marker.SPHERE_LIST

    line_strip.action = line_strip.ADD
    line_strip.color.r = 0.0
    line_strip.color.g = 0.0
    line_strip.color.a = 1.0
    line_strip.color.b = 1.0
    line_strip.scale.x = 1.1
    line_strip.scale.y = 1.1
    line_strip.scale.z = 1.1
    line_strip.pose.orientation.x = 0.0
    line_strip.pose.orientation.y = 0.0
    line_strip.pose.orientation.z = 0.0
    line_strip.pose.orientation.w = 1.0

    pub.publish(lane_sender)
    marker_pub.publish(line_strip)




def talker():
    global pub,marker_pub

    rospy.init_node('waypoint_shortener', anonymous=True)

    pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1)
    sub = rospy.Subscriber("/base_waypoints_raw",Lane,waypoints_callback,queue_size=1)
    sub_current_pose = rospy.Subscriber("/current_pose",PoseStamped,current_pose_callback,queue_size=1)
    marker_pub = rospy.Publisher("/original_waypoints",Marker,queue_size=1)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)



    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass