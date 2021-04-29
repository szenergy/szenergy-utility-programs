#!/usr/bin/env python

import rospy
import autoware_msgs.msg as aw_msgs
from playsound import playsound

buf = 0

def subscriber():
    sub = rospy.Subscriber("/vehicle_status", aw_msgs.VehicleStatus, callback_fuction)
    rospy.spin()

def callback_fuction(message):
    if (1 - message.drivemode + buf == 0):
        playsound("/home/zsoldos/szenergy_ws/src/szenergy-utility-programs/drivemode_sound/src/start_trimmed.mp3")
        global buf
        buf = 1
    elif (2 - message.drivemode - buf == 1):
        playsound("/home/zsoldos/szenergy_ws/src/szenergy-utility-programs/drivemode_sound/src/stop_trimmed.mp3")
        global buf
        buf = 0
    else: None

if __name__ == "__main__":
    rospy.init_node("teszt_sub")
    subscriber()
