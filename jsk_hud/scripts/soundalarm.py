#!/usr/bin/env python
import rospy
import rospkg
from jsk_rviz_plugins.msg import OverlayText
import time
from playsound import playsound
playing = False
timer_hw = time.time()
#timer_sw = time.time()
length = 0.02
def callback_hw(data):
    global playing
    global timer_hw
    global length
    if (data.text!=""):
        if not playing:
            playsound(rospack.get_path("jsk_hud") + "/audio/alarm.mp3")
            playing = True
            timer_hw = time.time()
    if time.time()-timer_hw > length:
        playing = False;

def callback_sw(data):
    global playing
    global timer_hw
    global length
    if (data.text!=""):
        if not playing:
            playsound(rospack.get_path("jsk_hud") + "/audio/alarm.mp3")
            playing = True
            timer_hw = time.time()
    if time.time()-timer_hw > length:
        playing = False;
    
def listener():

    rospy.init_node('hud_topic_alarm')
    rospy.Subscriber("/topic_error_hw", OverlayText, callback_hw)
    rospy.Subscriber("/topic_error_sw", OverlayText, callback_sw)
    rospy.spin()

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    rospy.loginfo("hud topic alarm node started")
    listener()
