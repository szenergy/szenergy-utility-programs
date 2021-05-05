#!/usr/bin/env python

import rospy
import rospkg
import autoware_msgs.msg as aw_msgs
from playsound import playsound

def subscriber():
    sub = rospy.Subscriber("/vehicle_status", aw_msgs.VehicleStatus, callback_fuction)
    rospy.spin()

def callback_fuction(message):
    global buf
    if (1 - message.drivemode + buf == 0):
        playsound(rospack.get_path("drivemode_sound") + "/src/start_trimmed.mp3")
        buf = 1
    elif (2 - message.drivemode - buf == 1):
        playsound(rospack.get_path("drivemode_sound") + "/src/stop_trimmed.mp3")
        buf = 0
    else: None

if __name__ == "__main__":
    buf = 0
    rospack = rospkg.RosPack()
    rospy.init_node("drivemode_sub")
    subscriber()