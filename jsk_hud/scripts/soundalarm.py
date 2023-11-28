#!/usr/bin/env python
import rospy
import rospkg
from jsk_rviz_plugins.msg import OverlayText
import time
from playsound import playsound
playing = False
timer = time.time()
length = 0.02
def callback(data):
    global playing
    global timer
    global length
    if data.text:
        if not playing:
            playsound(rospack.get_path("jsk_hud") + "/audio/alarm.mp3")
            playing = True
            timer = time.time()
    if time.time()-timer > length:
        playing = False;
    
def listener():

    rospy.init_node('hud_topic_alarm')
    rospy.Subscriber("/topic_error", OverlayText, callback)

    rospy.spin()

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    rospy.loginfo("hud topic alarm node started")
    listener()
