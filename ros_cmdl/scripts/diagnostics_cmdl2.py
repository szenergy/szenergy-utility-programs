#!/usr/bin/env python
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import geometry_msgs.msg as geomsg
import sensor_msgs.msg as senmsg
try:
    import novatel_gps_msgs.msg as novamsg
except:
    None
import autoware_msgs.msg as autowmsgs

import sys,random
from pynvml import *
import psutil
import colorama


class DataProcessor:

    def __init__(self):
        colorama.init()
        self.vstatus_sub = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus,self.vehicleStatusCallback)

        self.dstatus_sub = rospy.Subscriber("/gps/duro/status_string",rosmsg.String,self.duroRtkStatusCallBack)
        
        self.stwheel_sub = rospy.Subscriber("/wheel_angle_deg",rosmsg.Float32,self.steeringWheelCallBack)

        self.vspeed_sub = rospy.Subscriber("/vehicle_speed_kmph", rosmsg.Float32, self.speedCallBack)

        self.rspeed_sub = rospy.Subscriber("/ctrl_cmd", autowmsgs.ControlCommandStamped, self.refSpeedCallBack)

        try:
            self.nstatus_sub = rospy.Subscriber('/gps/nova/bestvel',novamsg.NovatelVelocity, self.novaRtkStatusCallBack)
        except:
            rospy.logwarn("no novatel_gps_msgs.msg custom messages built")
            self.nova_rtk = "NoNovaCustomMsg"

        self.leaf_driver_mode = ''
        self.leaf_speed = 0.0
        self.ref_speed = 0.0
        self.st_wheel_angle = 0.0

        self.duro_stime = rospy.Time.now()
        self.nova_stime = rospy.Time.now()
        self.duro_time = 0.0
        self.nova_time = 0.0
        self.duro_rtk = ''
        self.nova_rtk = ''

        self.ouster_left_ok = ''
        self.ouster_right_ok = ''
        self.velo_left_ok = ''
        self.velo_right_ok = ''
        self.sick_ok = ''
        self.zed_ok = ''

        self.gpu_usage = 0.0
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
    
    def getLeftOusterMsg(self):
        try:
            rospy.wait_for_message("/left_os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
            self.ouster_left_ok = "OK"
        except rospy.ROSException:
            self.ouster_left_ok = "\033[1;31mERR\033[0;0m"

    def getRightOusterMsg(self):
        try:
            rospy.wait_for_message("/right_os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
            self.ouster_right_ok = "OK"
        except rospy.ROSException:
            self.ouster_right_ok = "\033[1;31mERR\033[0;0m"

    def getLeftVeloMsg(self):
        try:
            rospy.wait_for_message("/velodyne_left/velodyne_points", senmsg.PointCloud2, timeout=0.2)
            self.velo_left_ok = "OK"
        except rospy.ROSException:
            self.velo_left_ok = "\033[1;31mERR\033[0;0m"  

    def getRightVeloMsg(self):
        try:
            rospy.wait_for_message("/velodyne_right/velodyne_points", senmsg.PointCloud2, timeout=0.2)
            self.velo_right_ok = "OK"
        except rospy.ROSException:
            self.velo_right_ok = "\033[1;31mERR\033[0;0m"  

    def getSickMsg(self):
        try:
            rospy.wait_for_message("/scan", senmsg.PointCloud2, timeout=0.2)
            self.sick_ok = "OK"
        except rospy.ROSException:
            self.sick_ok = "\033[1;31mERR\033[0;0m"

    def getZedMsg(self):
        try:
            rospy.wait_for_message("/zed_node/left/image_rect_color/compressed", senmsg.CompressedImage, timeout=0.2)
            self.zed_ok = "OK"
        except rospy.ROSException:
            self.zed_ok = "\033[1;31mERR\033[0;0m"


    def getGPUUsage(self):
        nvmlInit()

        deviceCount = nvmlDeviceGetCount()
        for i in range(deviceCount):
            handle = nvmlDeviceGetHandleByIndex(i)
            res = nvmlDeviceGetUtilizationRates(handle)
            self.gpu_usage = res.gpu

        nvmlShutdown()

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

    def printAngleBar(self,angle):
        result = "|"
        st_angle = 0
        if angle < 0.0:
            st_angle = -12 if (int)(angle) < -12 else (int)(angle)
        else:
            st_angle = 12 if (int)(angle) > 12 else (int)(angle)

        if angle < 0.0:
            result += "."*(13 + st_angle-1)
            result += "-"*(13-(13 + st_angle))

            result += "o"
            while len(result) <= 25:
                result += "."

        elif angle == 0.0:
            result += "............o............"

        else:
            result += "."*12
            result += "o"
            result += "-"*(13+st_angle-13)
            
            while len(result) <= 25:
                result += "."
        
        result += "|" + self.twoDecimal(angle)

        return result

    def loadLimit(self,s):
        if s >= 80:
            return '\033[1;31m{:.2f}\033[0;0m'.format(s)
        else:
            return '{:.2f}'.format(s)


    def printLayout(self):
        if self.iterator == 0:
            self.data_processor.getLeftOusterMsg()
        if self.iterator == 8:
            self.data_processor.getRightOusterMsg()
        if self.iterator == 16:
            self.data_processor.getLeftVeloMsg()
        if self.iterator == 24:
            self.data_processor.getRightVeloMsg()
        if self.iterator == 32:
            self.data_processor.getSickMsg()
        if self.iterator == 40:
            self.data_processor.getZedMsg()
            self.iterator = -1
        self.iterator += 1
        self.setCursorPosition()
        self.data_processor.getCPUUsage()
        self.data_processor.getGPUUsage()
        self.data_processor.getRamUsage()

        duro_msg = 'Duro: ' + self.data_processor.duro_rtk + " since:" + self.twoDecimal(self.data_processor.duro_time) + " s"

        novatel_msg = "Novatel: " + self.data_processor.nova_rtk + " since:" +self.twoDecimal(self.data_processor.nova_time) + " s"
        left_ouster_msg ="OusterLeft: " + self.data_processor.ouster_left_ok
        right_ouster_msg ="OusterRight: " + self.data_processor.ouster_right_ok
        left_velo_msg = "VeloLeft: " + self.data_processor.velo_left_ok
        right_velo_msg = "VeloRight: " + self.data_processor.velo_right_ok
        zed_msg = "Zed: " + self.data_processor.zed_ok
        sick_msg = "SICK: " + self.data_processor.sick_ok
        speed_msg = "Speed: " + self.twoDecimal(self.data_processor.leaf_speed) +"km/h ref: " + self.twoDecimal(self.data_processor.ref_speed) +" km/h"
        st_wheel_msg = self.printAngleBar(self.data_processor.st_wheel_angle)
        driver_mode = "Driver mode: " + self.data_processor.leaf_driver_mode
        cpu_usage = "CPU Load: " + self.loadLimit(self.data_processor.cpu_usage) + "%"
        process ="GPU Load: " + self.loadLimit(self.data_processor.gpu_usage) + "%"
        ram_alloc ="RAM allocation:" +self.twoDecimal(self.data_processor.ram_usage) + " MB"
        self.printData(duro_msg, novatel_msg)
        self.printData(left_ouster_msg, right_ouster_msg)
        self.printData(left_velo_msg, right_velo_msg)
        self.printData(zed_msg, sick_msg)
        self.printData(speed_msg, st_wheel_msg)
        self.printSingleData(driver_mode)
        self.printSingleData(cpu_usage)
        self.printSingleData(process)
        self.printSingleData(ram_alloc)

if __name__ == '__main__':
    rospy.init_node('sensor_diagnostic')

    rate = rospy.Rate(20)

    lp = LayoutPrinter()
   
    while not rospy.is_shutdown():
        lp.printLayout()
        rate.sleep()
        


