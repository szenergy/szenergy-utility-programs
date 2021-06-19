#!/usr/bin/env python

import rospy
import rospkg
import novatel_gps_msgs as nova_msgs
from playsound import playsound

fixed = 0

def subscriber():
    rospy.Subscriber("/velocity_type", nova_msgs.NovatelVelocity, callback_fuction)
    rospy.spin()

def callback_fuction(message):
    global fixed
    if (message.velocity_type == 'INS_RTKFIXED') and (fixed == 0):
        playsound(rospack.get_path("drivemode_sound") + "/src/rtk_fixed.mp3")
        fixed = 1
    elif (message.velocity_type != 'INS_RTKFIXED') and (fixed == 1):
        playsound(rospack.get_path("drivemode_sound") + "/src/rtk_not_fixed.mp3")
        fixed = 0

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    rospy.init_node("velotype_sub")
    rospy.loginfo("drivemode_sound velotype_sound node started")
    subscriber()