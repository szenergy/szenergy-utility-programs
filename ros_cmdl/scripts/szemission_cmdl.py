#!/usr/bin/env python
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import geometry_msgs.msg as geomsg
import sensor_msgs.msg as senmsg
import autoware_msgs.msg as autowmsgs

import sys
import colorama

class DataProcessor:

    def __init__(self):
        colorama.init()
        self.vstatus_sub = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus,self.vehicleStatusCallback)

        self.dstatus_sub = rospy.Subscriber("/gps/duro/status_string",rosmsg.String,self.duroRtkStatusCallBack)
        
        self.stwheel_sub = rospy.Subscriber("/wheel_angle_deg",rosmsg.Float32,self.steeringWheelCallBack)

        self.vspeed_sub = rospy.Subscriber("/vehicle_speed_kmph", rosmsg.Float32, self.speedCallBack)

        self.rspeed_sub = rospy.Subscriber("/ctrl_cmd", autowmsgs.ControlCommandStamped, self.refSpeedCallBack)

        self.leaf_driver_mode = ''
        self.leaf_speed = 0.0
        self.ref_speed = 0.0
        self.st_wheel_angle = 0.0

        self.duro_stime = rospy.Time.now()
        self.duro_time = 0.0
        self.duro_rtk = ''

        self.ouster_ok = ''
        self.sick_ok = ''
        self.zed_ok = ''

        self.cpu_usage = 0.0
        self.ram_usage = 0.0

    def vehicleStatusCallback(self,msg):
        if msg.drivemode == 0:
            self.leaf_driver_mode = "DRIVER"
        elif msg.drivemode == 1:
            self.leaf_driver_mode = "AUTONOMOUS"
        else:
            self.leaf_driver_mode = "\033[1;31mUNDEF\033[0;0m"

    def duroRtkStatusCallBack(self,msg):
        if msg.data != self.duro_rtk:
            self.duro_stime = rospy.Time.now()
        self.duro_time = (rospy.Time.now() - self.duro_stime).to_sec()
        self.duro_time = 0.0 if self.duro_time <= 0 else self.duro_time
        self.duro_rtk = msg.data

    def novaRtkStatusCallBack(self,msg):
        if msg.velocity_type != self.nova_rtk:
            self.nova_stime = rospy.Time.now()
    
        self.nova_time = (rospy.Time.now() - self.nova_stime).to_sec()
        self.nova_time = 0.0 if self.nova_time <= 0 else self.nova_time
        self.nova_rtk = msg.velocity_type
  
    def steeringWheelCallBack(self,msg):
        self.st_wheel_angle = msg.data
    
    def speedCallBack(self,msg):
        self.leaf_speed = msg.data

    def refSpeedCallBack(self,msg):
        self.ref_speed = msg.cmd.linear_velocity
    
    def getOusterMsg(self):
        try:
            rospy.wait_for_message("/os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
            self.ouster_ok = "OK"
        except rospy.ROSException:
            self.ouster_ok = "\033[1;31mERR\033[0;0m" 

    def getSickMsg(self):
        try:
            rospy.wait_for_message("/scan", senmsg.LaserScan, timeout=0.2)
            self.sick_ok = "OK"
        except rospy.ROSException:
            self.sick_ok = "\033[1;31mERR\033[0;0m"

    def getZedMsg(self):
        try:
            rospy.wait_for_message("/zed_node/left/image_rect_color/compressed", senmsg.CompressedImage, timeout=0.2)
            self.zed_ok = "OK"
        except rospy.ROSException:
            self.zed_ok = "\033[1;31mERR\033[0;0m"

    def getCPUUsage(self):
        self.cpu_usage = psutil.cpu_percent()
    
    def getRamUsage(self):
        MB = 1024*1024
        mem = psutil.virtual_memory()
        self.ram_usage = mem.used/MB


class LayoutPrinter:
    def __init__(self):
        self.data_processor = DataProcessor()
        self.height = 9
        self.width = 110
        self.iterator = 0
        self.setLineHeight()

    def setLineHeight(self):
        sys.stdout.write("\n" * self.height) # Make sure we have space to draw the bars

    def setCursorPosition(self):
        sys.stdout.write(u"\u001b[1000D") # Move left
        sys.stdout.write(u"\u001b[" + str(self.height) + "A") # Move up

    def printData(self,s1, s2):
        half_width = self.width // 2
        
        sep1 = " "*(half_width - len(s1))
        sep2 = " "*(half_width - len(s2))

        print(s1+sep1+s2+sep2)

    def printSingleData(self,s):
        sep = " "*(self.width - len(s))
        print(s+sep)

    def twoDecimal(self,s):
        return "{:.2f}".format(s)



    def printLayout(self):
        if self.iterator == 0:
            self.data_processor.getOusterMsg()
        if self.iterator == 8:
            self.data_processor.getSickMsg()
        if self.iterator == 16:
            self.data_processor.getZedMsg()
            self.iterator = -1
        self.iterator += 1
        self.setCursorPosition()

        duro_msg = 'Duro: ' + self.data_processor.duro_rtk + " since:" + self.twoDecimal(self.data_processor.duro_time) + " s"

        left_ouster_msg ="Ouster State: " + self.data_processor.ouster_ok
        zed_msg = "Zed State: " + self.data_processor.zed_ok
        sick_msg = "SICK State: " + self.data_processor.sick_ok
        speed_msg = "Speed: " + self.twoDecimal(self.data_processor.leaf_speed) +"km/h ref: " + self.twoDecimal(self.data_processor.ref_speed) +" km/h"
        st_wheel_msg = "Steering Wheel Angle:"+self.twoDecimal(self.data_processor.st_wheel_angle)
        driver_mode = "Driver mode: " + self.data_processor.leaf_driver_mode
        cpu_usage = "CPU Load: " + self.twoDecimal(self.data_processor.cpu_usage) + "%"
        ram_alloc ="RAM allocation:" +self.twoDecimal(self.data_processor.ram_usage) + " MB"
        self.printSingleData(duro_msg)
        self.printSingleData(left_ouster_msg)
        self.printSingleData(zed_msg)
        self.printSingleData(sick_msg)
        self.printSingleData(speed_msg)
        self.printSingleData(st_wheel_msg)
        self.printSingleData(driver_mode)
        self.printSingleData(cpu_usage)
        self.printSingleData(ram_alloc)



if __name__ == '__main__':
    rospy.init_node('szemission_diagnostics')

    rate = rospy.Rate(20)

    lp = LayoutPrinter()
   
    while not rospy.is_shutdown():
        lp.printLayout()
        rate.sleep()