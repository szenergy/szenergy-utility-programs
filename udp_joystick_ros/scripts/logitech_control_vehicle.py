#!/usr/bin/env python

# This is NOT an UDP-based Android control, it used a Logitech or any other game pad

"""
### useful commands (http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

$ lsusb
Bus 001 Device 004: ID 046d:c219 Logitech, Inc. Cordless RumblePad 2
$ ls -l /dev/input/js0 
$ sudo chmod a+rw /dev/input/js0
$ rosparam set joy_node/dev "/dev/input/js0"

"""


import rospy
import geometry_msgs.msg as geomsg
import autoware_msgs.msg as auwmsg
import sensor_msgs.msg as sensmsg
import numpy as np
import time
import threading

class GamePadJoystick:
    start_controller_state = np.array([0, 0, 0, 0])

    def __init__(self):
        self.pub_tw = rospy.Publisher("ctrl_cmd", auwmsg.ControlCommandStamped, queue_size=10)
        rospy.loginfo("Game pad based publishing: ctrl_cmd [autoware_msgs/ControlCommandStamped]")
        self.speed_j = 0.0
        self.angl_j = 0.0

    def start_callback(self):
        self.sub_joy = rospy.Subscriber("joy", sensmsg.Joy, self.joy_callback)

    def stop_callback(self):
        self.sub_joy.unregister()

    def start_thread(self):
        thread = threading.Thread(target = self.loop)
        self._stop = threading.Event()
        thread.start()

    def stop_thread(self):
        self._stop.set()

    def loop(self):
        msg_aw = auwmsg.ControlCommandStamped()
        r = rospy.Rate(20) # 20hz
        while(not self._stop.isSet()):
            if self.speed_j >= 0.001:
                msg_aw.cmd.linear_velocity = self.speed_j * 20
            else:
                msg_aw.cmd.linear_velocity = 0.0
            msg_aw.cmd.steering_angle = self.angl_j * 0.5
            msg_aw.header.frame_id = "logitech_game_pad"
            msg_aw.header.stamp = rospy.Time.now()
            self.pub_tw.publish(msg_aw)
            r.sleep()

    def joy_callback(self, mgs_joy):
        self.speed_j = mgs_joy.axes[3]
        self.angl_j = mgs_joy.axes[0]

if __name__ == "__main__":

    rospy.init_node("joy_control", disable_signals=True)
    try:
        gpj = GamePadJoystick()
        gpj.start_callback()
        gpj.start_thread()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:   
        gpj.stop_callback()
        gpj.stop_thread()
        rospy.loginfo("Shutting down ros node...")
