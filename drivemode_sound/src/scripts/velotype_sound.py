#!/usr/bin/env python

import rospy
import rospkg
try:
    import novatel_gps_msgs.msg as nova_msgs
except:
    print("Download and catkin build novatel_gps_msgs")
from playsound import playsound

# rtk_fstate true if in fixed rtk state, prev is the previous state
rtk_fstate = False; prev_rtk_fstate = False

def subscriber():
    rospy.Subscriber("/velocity_type", nova_msgs.NovatelVelocity, callback_fuction)
    rospy.spin()

def callback_fuction(message):
    global rtk_fstate, prev_rtk_fstate
    if (message.velocity_type == 'INS_RTKFIXED'):
        rtk_fstate = True
    elif (message.velocity_type != 'INS_RTKFIXED'):
        rtk_fstate = False
    if rtk_fstate != prev_rtk_fstate:
        if rtk_fstate == True:
            playsound(rospack.get_path("drivemode_sound") + "/src/rtk_fixed.mp3")
        else:
            playsound(rospack.get_path("drivemode_sound") + "/src/rtk_not_fixed.mp3")
        prev_rtk_fstate = rtk_fstate

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    rospy.init_node("velotype_sub")
    rospy.loginfo("drivemode_sound velotype_sound node started")
    subscriber()