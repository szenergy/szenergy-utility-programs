#!/usr/bin/env python3

import rospy
import rospkg
import autoware_msgs.msg as aw_msgs
from playsound import playsound

buf = 0

def subscriber():
    rospy.Subscriber("/vehicle_status", aw_msgs.VehicleStatus, callback_fuction)
    rospy.spin()

def callback_fuction(message):
    global buf
    if (1 - message.drivemode + buf == 0):
        playsound(rospack.get_path("drivemode_sound") + "/src/airbus_ap_engage.mp3")
        buf = 1
    elif (2 - message.drivemode - buf == 1):
        playsound(rospack.get_path("drivemode_sound") + "/src/airbus_ap_disengage.mp3")
        buf = 0


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    rospy.init_node("drivemode_sub")
    rospy.loginfo("drivemode_sound play_sound node started")
    playsound(rospack.get_path("drivemode_sound") + "/src/wood_plank_flicks.mp3")
    subscriber()