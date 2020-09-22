#!/usr/bin/env python


import rospy
import numpy as np
import tf
from tf.transformations import *

gps_x=rospy.get_param('/pcd_transform_quaterion/gps_quat_x')
gps_y=rospy.get_param('/pcd_transform_quaterion/gps_quat_y')
gps_z=rospy.get_param('/pcd_transform_quaterion/gps_quat_z')
gps_w=rospy.get_param('/pcd_transform_quaterion/gps_quat_w')

ndt_x=rospy.get_param('/pcd_transform_quaterion/ndt_pose_quat_x')
ndt_y=rospy.get_param('/pcd_transform_quaterion/ndt_pose_quat_y')
ndt_z=rospy.get_param('/pcd_transform_quaterion/ndt_pose_quat_z')
ndt_w=rospy.get_param('/pcd_transform_quaterion/ndt_pose_quat_w')


q1_inverse=np.zeros(4)

q1=np.array([ndt_x,ndt_y,ndt_z,ndt_w])


q2=np.array([gps_x,gps_y,gps_z,gps_w])

q1_inverse[0] = q1[0]
q1_inverse[1] = q1[1]
q1_inverse[2] = q1[2]
q1_inverse[3] = -q1[3]

q_rel=tf.transformations.quaternion_multiply(q2,q1_inverse)

quat=tf.transformations.euler_from_quaternion(q_rel)

roll=np.degrees(quat[0])
pitch=np.degrees(quat[1])
yaw=np.degrees(quat[2])

print(roll,pitch,yaw)


