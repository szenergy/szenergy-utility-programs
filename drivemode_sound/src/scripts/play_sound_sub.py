#!/usr/bin/env python3

import rospy
import rospkg
import autoware_msgs.msg as aw_msgs
from playsound import playsound

buf = 0
ctrl_cmd = None
has_ctrl_cmd = False

def subscriber():
    rospy.Subscriber("/vehicle_status", aw_msgs.VehicleStatus, callback_fuction)
    rospy.Subscriber("/ctrl_cmd", aw_msgs.ControlCommandStamped, ctrl_cmd_callback)
    rospy.spin()

def ctrl_cmd_callback(message):
    global ctrl_cmd

    ctrl_cmd = message

def ctrl_cmd_sound_player(req):
    global ctrl_cmd

    if ctrl_cmd is not None:
        print(rospy.Time.now() - ctrl_cmd.header.stamp)
        if (rospy.Time.now() - ctrl_cmd.header.stamp) > rospy.Duration(1.0):
            playsound(rospack.get_path("drivemode_sound") + "/src/master_caution.mp3")


def callback_fuction(message):
    global buf
    if (1 - message.drivemode + buf == 0):
        playsound(rospack.get_path("drivemode_sound") + "/src/airbus_ap_engage.mp3")
        buf = 1
    elif (2 - message.drivemode - buf == 1):
        playsound(rospack.get_path("drivemode_sound") + "/src/airbus_ap_disengage.mp3")
        buf = 0
    if abs(message.angle) > 0.15708:
        playsound(rospack.get_path("drivemode_sound") + "/src/h-bank-angle.mp3")


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    rospy.init_node("drivemode_sub")
    rospy.loginfo("drivemode_sound play_sound node started")
    playsound(rospack.get_path("drivemode_sound") + "/src/wood_plank_flicks.mp3")
    # start ctrl_cmd sound player on a separate thread
    rospy.Timer(rospy.Duration(0.1), ctrl_cmd_sound_player)

    subscriber()