#!/usr/bin/env python
from __future__ import print_function
import rospy, rostopic, roslaunch
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import geometry_msgs.msg as geomsg
import sensor_msgs.msg as senmsg
import novatel_gps_msgs.msg as novamsg
import autoware_msgs.msg as autowmsgs
import numpy as np
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea
import os.path

class PlotHandler(object):
    def __init__(self, leaf):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.leaf = leaf
        self.app = qtgqt.QtGui.QApplication([])

    def initializePlot(self):
        self.first_run = True
        self.paused = False
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        darkyellow = (224, 166, 58); darkyellowB = pg.mkBrush(224, 166, 58, 200);
        self.win.setWindowTitle("Leaf control 1")
        self.win.resize(700, 600)
        self.win.setCentralWidget(area)
        dock1def = darea.Dock("Default", size = (1,1))  # give this dock minimum possible size
        dock2oth = darea.Dock("Others", size = (1,1))  # give this dock minimum possible size
        dock3ctr = darea.Dock("Control", size = (1,1))  # give this dock minimum possible size
        dock4gps = darea.Dock("2 Gps visualization", size = (500,400)) # size is only a suggestion
        area.addDock(dock2oth, "left")
        area.addDock(dock1def, "above", dock2oth)
        area.addDock(dock3ctr, "bottom", dock1def)
        area.addDock(dock4gps, "bottom", dock3ctr)
        dhLabel = qtgqt.QtGui.QLabel("Duro:"); dhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dhLabel.setAlignment(pg.QtCore.Qt.AlignRight); dhLabel.setFixedSize(50, 25)
        dsLabel = qtgqt.QtGui.QLabel("Duro:"); dsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dsLabel.setAlignment(pg.QtCore.Qt.AlignRight); dsLabel.setFixedSize(50, 25)
        nhLabel = qtgqt.QtGui.QLabel("Nova:"); nhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        nsLabel = qtgqt.QtGui.QLabel("Nova:"); nsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nsLabel.setAlignment(pg.QtCore.Qt.AlignRight)     
        self.duroHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.duroRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.novaHzLabel = qtgqt.QtGui.QLabel("  **.* Hz")
        self.novaRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.pauseSensorReadBtn = qtgqt.QtGui.QPushButton("Pause")
        self.savePoseBtn = qtgqt.QtGui.QPushButton("Save")
        self.allSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start AllSensor")
        self.loadWaypointBtn = qtgqt.QtGui.QPushButton("Load waypoints")
        self.saveWaypointBtn = qtgqt.QtGui.QPushButton("Save waypoints")
        self.selectFileBtn = qtgqt.QtGui.QPushButton("...")
        self.selectFileBtn.setMaximumWidth(22)
        self.setFileBtn = qtgqt.QtGui.QPushButton("Set")
        self.setFileBtn.setMaximumWidth(22)
        self.speedLabel = qtgqt.QtGui.QLabel(" **.* Km/h")
        self.angleLabel = qtgqt.QtGui.QLabel(" **.* rad")
        self.csvLabel = qtgqt.QtGui.QLabel("none"); self.csvLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        self.csvTextbox = qtgqt.QtGui.QLineEdit()
        self.allSensorLaunched = False
        self.waypointLoaded = False
        self.waypointSaving = False
        widg1def = pg.LayoutWidget()
        widg1def.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1def.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock1def.addWidget(widg1def)
        self.novaHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.novaRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.duroHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(224, 166, 58)")
        self.duroRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(244, 166, 58)")
        self.angleLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.speedLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.csvLabel.setStyleSheet("font: 10pt; color: rgb(244, 166, 58)")
        sickLabel = qtgqt.QtGui.QLabel("Sick:"); sickLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); sickLabel.setAlignment(pg.QtCore.Qt.AlignRight); sickLabel.setFixedSize(50, 25)
        olhLabel = qtgqt.QtGui.QLabel("OusterLeft:"); olhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); olhLabel.setAlignment(pg.QtCore.Qt.AlignRight); olhLabel.setFixedSize(80, 25)
        orhLabel = qtgqt.QtGui.QLabel("OusterRight:"); orhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); orhLabel.setAlignment(pg.QtCore.Qt.AlignRight); orhLabel.setFixedSize(95, 25)
        vlhLabel = qtgqt.QtGui.QLabel("VeloLeft:"); vlhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); vlhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        vrhLabel = qtgqt.QtGui.QLabel("VeloRight:"); vrhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); vrhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        self.sickOkLabel = qtgqt.QtGui.QLabel("**")
        self.sickOkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(224, 166, 58)")
        self.ousterLefLabel = qtgqt.QtGui.QLabel(" **")
        self.ousterLefLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.ousterRigLabel = qtgqt.QtGui.QLabel(" **")
        self.ousterRigLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.veloLef = qtgqt.QtGui.QLabel(" **")
        self.veloLef.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")
        self.veloRig = qtgqt.QtGui.QLabel(" **")
        self.veloRig.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")      
        # default
        widg1def.addWidget(dsLabel, row=1, col=6)
        widg1def.addWidget(nsLabel, row=2, col=6)
        widg1def.addWidget(self.duroRtkLabel, row=1, col=7)
        widg1def.addWidget(self.novaRtkLabel, row=2, col=7)
        widg1def.addWidget(self.speedLabel, row=3, col=2)
        widg1def.addWidget(self.angleLabel, row=3, col=4)
        widg1def.addWidget(self.pauseSensorReadBtn, row=4, col=7)
        widg1def.addWidget(olhLabel, row=1, col=1)
        widg1def.addWidget(orhLabel, row=1, col=3)
        widg1def.addWidget(vlhLabel, row=2, col=1)
        widg1def.addWidget(vrhLabel, row=2, col=3)
        widg1def.addWidget(sickLabel, row=4, col=1)
        widg1def.addWidget(self.sickOkLabel, row=4, col=2)
        widg1def.addWidget(self.ousterLefLabel, row=1, col=2)
        widg1def.addWidget(self.ousterRigLabel, row=1, col=4)
        widg1def.addWidget(self.veloLef, row=2, col=2)
        widg1def.addWidget(self.veloRig, row=2, col=4)
        self.zedOkLabel = qtgqt.QtGui.QLabel("**")
        self.zedOkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(123, 64, 133)")
        zhLabel = qtgqt.QtGui.QLabel("Zed:"); zhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); zhLabel.setAlignment(pg.QtCore.Qt.AlignRight); zhLabel.setFixedSize(50, 25)
        widg1def.addWidget(zhLabel, row=3, col=6)
        widg1def.addWidget(self.zedOkLabel, row=3, col=7)

        self.state = None
        self.widgGps = pg.PlotWidget(title="Gps difference")
        self.widgGps.setAspectLocked(True)
        self.pltGpsOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = blueB)
        self.pltLeafOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widgGps.showGrid(x=True, y=True)
        self.widgGps.addItem(self.pltGpsOdom)
        self.widgGps.addItem(self.pltLeafOdom)
        dock4gps.addWidget(self.widgGps)
        self.pauseSensorReadBtn.clicked.connect(self.pauseSensorRead)
        self.savePoseBtn.clicked.connect(self.saveToCsv)
        self.allSensorLaunchBtn.clicked.connect(self.startAllSensor)
        self.loadWaypointBtn.clicked.connect(self.loadCsv)
        self.saveWaypointBtn.clicked.connect(self.saveCsv)
        self.selectFileBtn.clicked.connect(self.selectCsv)
        self.setFileBtn.clicked.connect(self.waypointSaveSet)
        
        self.tGps = pg.TextItem(text = "Gps", color = blue)
        self.tLeaf = pg.TextItem(text = "Leaf odom", color = red)
        self.tstart = pg.TextItem(text = "Start", color = white)

        
        dock2oth.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.drawCircle(self.widgGps)
        
        # control
        widg3ctr = pg.LayoutWidget()
        dock3ctr.setStyleSheet("background-color: rgb(18, 20, 23);")
        widg3ctr.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        widg3ctr.addWidget(self.allSensorLaunchBtn, row=1, col=1)
        widg3ctr.addWidget(self.loadWaypointBtn, row=2, col=1)
        widg3ctr.addWidget(self.saveWaypointBtn, row=3, col=1)
        widg3ctr.addWidget(self.csvLabel, row=2, col=2)
        widg3ctr.addWidget(self.selectFileBtn, row=2, col=3)
        widg3ctr.addWidget(self.csvTextbox, row=3, col=2)
        widg3ctr.addWidget(self.setFileBtn, row=3, col=3)
        dock3ctr.addWidget(widg3ctr)
        try:
            current_file = rospy.get_param("waypoint_file_name")
            self.csvLabel.setText(os.path.basename(str(current_file)))
        except:
            self.csvLabel.setText("No waypoint_file_name param")
        self.win.show()

    def updatePose(self):
        if self.paused == False:
            self.duroRtkLabel.setText(self.leaf.duro_rtk)
            self.novaRtkLabel.setText(self.leaf.nova_rtk)
            self.speedLabel.setText("%5.1f Km/h" % (self.leaf.leaf_speed * 3.6))
            self.angleLabel.setText("%5.3f rad" % (self.leaf.leaf_angl))
            self.ousterLefLabel.setText(self.leaf.ouster_lef_ok)
            self.ousterRigLabel.setText(self.leaf.ouster_rig_ok)
            self.sickOkLabel.setText(self.leaf.sick_ok)
            self.zedOkLabel.setText(self.leaf.zed_ok)
            self.veloLef.setText(self.leaf.velo_lef_ok)
            self.veloRig.setText(self.leaf.velo_rig_ok)
            self.pltGpsOdom.setPoints(self.leaf.pose_diff.x, self.leaf.pose_diff.y)    

    def startAllSensor(self):
        if self.allSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = "/home/nvidia/leaf_ws/src/nissan_leaf_ros/nissan_bringup/launch/nissan.leaf.bringup.2020.A.launch"
            self.launchAS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchAS.start()
            rospy.loginfo(launchStr + "started")
            self.allSensorLaunchBtn.setText("Stop AllSensor")
        else:
            self.launchAS.shutdown()
            rospy.loginfo("All sensor stopped.....")
            self.allSensorLaunchBtn.setText("Start AllSensor")
        self.allSensorLaunched = not self.allSensorLaunched

    def loadCsv(self):
        if self.waypointLoaded is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = "/home/nvidia/leaf_ws/src/nissan_leaf_ros/nissan_bringup/launch/demo.waypoint.follow.launch"
            self.launchLC = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchLC.start()
            rospy.loginfo(launchStr + "started")
            self.loadWaypointBtn.setText("UnLoad waypoints")
        else:
            self.launchLC.shutdown()
            rospy.loginfo("waypoint unloaded")
            self.loadWaypointBtn.setText("Load waypoints")
        self.waypointLoaded = not self.waypointLoaded

    def saveCsv(self):
        if self.waypointSaving is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = "/home/nvidia/leaf_ws/src/nissan_leaf_ros/nissan_bringup/launch/waypoint.saver.launch"
            self.launchSC = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchSC.start()
            rospy.loginfo(launchStr + "started")
            rospy.loginfo("waypoint saving started")
            self.saveWaypointBtn.setText("Finish waypoints")
        else:
            self.launchSC.shutdown()
            rospy.loginfo("waypoint saving finished")
            self.saveWaypointBtn.setText("Save saveing wayp")
        self.waypointSaving = not self.waypointSaving    

    def waypointSaveSet(self):
        new_csv = "/mnt/storage_1tb/waypoint/" + str(self.csvTextbox.text())
        rospy.set_param("/waypoint_saver/save_filename", new_csv)
        #print(new_csv)

    def selectCsv(self):
        #fname = qtgqt.QtGui.QFileDialog.getOpenFileName(caption="Open file", directory="/mnt/storage_1tb/waypoint",filter="CSV files (*.csv)")
        dlg = qtgqt.QtGui.QFileDialog()
        dlg.setFileMode(qtgqt.QtGui.QFileDialog.AnyFile)
        dlg.setFilter("Text files (*.csv)")
        dlg.setDirectory("/mnt/storage_1tb/waypoint")
        filenames = qtgqt.QtCore.QStringList()
        if dlg.exec_():
            filenames = dlg.selectedFiles()
            rospy.set_param("waypoint_file_name", str(filenames[0]))
            self.csvLabel.setText(os.path.basename(str(filenames[0])))
            #f = open(filenames[0], 'r')
            #with f:
            #    data = f.read()
            #    print(data)

    def pauseSensorRead(self):
        self.paused = not self.paused
        if self.paused:
            self.pauseSensorReadBtn.setText("UnPause")
            self.ousterLefLabel.setText("paused")
        else:
            self.pauseSensorReadBtn.setText("Pause")

    def drawCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 8, pen = pg.mkPen(None), brush = pg.mkBrush(80, 80, 80, 200))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.1)) * 1.7
        y = np.cos(np.arange(0, np.pi*2, 0.1)) * 1.7
        circle.addPoints(x, y)

    def saveToCsv(self):
        #rospy.loginfo(self.leaf.pose_duro.x)
        f = open("gps.txt", "a+")
        data_to_save = np.array([self.leaf.pose_duro.x, self.leaf.pose_duro.y, self.leaf.pose_nova.x, self.leaf.pose_nova.y])
        np.savetxt(f, data_to_save, fmt="%.8f", delimiter=",", newline=" ")
        f.write("\n")
        f.close()

class Point2D(object):
    x = 0; y = 0
    def __init__(self, x = 0, y = 0):
        self.x = np.array([x])
        self.y = np.array([y])
    def abs(self):
        return (self.x**2 + self.y**2)**0.5
    def __add__(self, p):
        return Point2D(self.x + p.x, self.y + p.y)
    def __sub__(self, p):
        return Point2D(self.x - p.x, self.y - p.y)        

class LeafSubscriber(object):
    sub_pos = True; sub_pos_pre = True
    leaf_speed = 0.0; leaf_angl = 0.0
    pose_diff = Point2D(); pose_duro = Point2D(); pose_nova = Point2D()


    def __init__(self):
        self.registerPose()
        self.ouster_lef_ok = "-"
        self.ouster_rig_ok = "-"
        self.zed_ok = "-"
        self.velo_lef_ok = "-"
        self.velo_rig_ok = "-"
        self.sick_ok = "-"
        self.duro_rtk = "-"
        self.nova_rtk = "-"
    
    def slowGetMsg(self):
        if self.stop_slow is False:
            try:
                rospy.wait_for_message("/left_os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
                self.ouster_lef_ok = "OK"
            except rospy.ROSException, e:
                self.ouster_lef_ok = "ERR"
            try:
                rospy.wait_for_message("/right_os1/os1_cloud_node/points", senmsg.PointCloud2, timeout=0.2)
                self.ouster_rig_ok = "OK"
            except rospy.ROSException, e:
                self.ouster_rig_ok = "ERR"
            try:
                rospy.wait_for_message("/velodyne_left/velodyne_points", senmsg.PointCloud2, timeout=0.2)
                self.velo_lef_ok = "OK"
            except rospy.ROSException, e:
                self.velo_lef_ok = "ERR"  
            try:
                rospy.wait_for_message("/velodyne_right/velodyne_points", senmsg.PointCloud2, timeout=0.2)
                self.velo_rig_ok = "OK"
            except rospy.ROSException, e:
                self.velo_rig_ok = "ERR"  
            try:
                rospy.wait_for_message("/cloud", senmsg.PointCloud2, timeout=0.2)
                self.sick_ok = "OK"
            except rospy.ROSException, e:
                self.sick_ok = "ERR"                   
            try:
                rospy.wait_for_message("/zed_node/stereo/image_rect_color", senmsg.Image, timeout=0.2)
                self.zed_ok = "OK"
            except rospy.ROSException, e:
                self.zed_ok = "ERR"            

    def registerPose(self):
        self.stop_slow = False
        self.p2 = rospy.Subscriber("/gps/duro/current_pose", geomsg.PoseStamped, self.duroPoseCallBack)
        self.p3 = rospy.Subscriber("/gps/nova/current_pose", geomsg.PoseStamped, self.novaPoseCallBack)
        self.p4 = rospy.Subscriber("/gps/duro/status_string", rosmsg.String, self.duroRtkStatusCallBack)
        self.p5 = rospy.Subscriber("/gps/nova/bestvel", novamsg.NovatelVelocity, self.novaRtkStatusCallback)
        self.p6 = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus, self.vehicleStatusCallback)
    
    def unregisterPose(self):
        self.stop_slow = True
        self.p0.unregister()
        self.p1.unregister()
        self.p2.unregister()
        self.p3.unregister()
        self.p4.unregister()
        self.p5.unregister()

        
    def novaPoseCallBack(self, msg):
        self.pose_nova.x = msg.pose.position.x
        self.pose_nova.y = msg.pose.position.y

    def duroPoseCallBack(self, msg):
        self.pose_duro.x = msg.pose.position.x
        self.pose_duro.y = msg.pose.position.y
        self.pose_diff = self.pose_duro -self.pose_nova

    def vehicleStatusCallback(self, msg):
        self.leaf_angl = msg.angle
        self.leaf_speed = msg.speed

    def novaRtkStatusCallback(self, msg):
        self.nova_rtk = msg.velocity_type

    def duroRtkStatusCallBack(self, msg):
        self.duro_rtk = msg.data

    def handleRegistering(self):      
        if self.sub_pos != self.sub_pos_pre: # if subscribe / unsubscribe
            rospy.loginfo("Subscribed to pose topics: ", self.sub_pos)
            self.sub_pos_pre = self.sub_pos
            if self.sub_pos == True:
                self.registerPose()
            elif self.sub_pos == False:
                self.unregisterPose()  
  

if __name__ == "__main__":
    import sys
    rospy.loginfo("Leaf control 1 started... ")
    rospy.init_node("leafcontrol", anonymous=True)
    leafSub = LeafSubscriber()
    ph = PlotHandler(leafSub)
    ph.initializePlot()
    timerSlowSub = qtgqt.QtCore.QTimer()
    timerSlowSub.timeout.connect(leafSub.slowGetMsg)
    timerSlowSub.start(3000)
    timerSubUnsub = qtgqt.QtCore.QTimer()
    timerSubUnsub.timeout.connect(leafSub.handleRegistering)
    timerSubUnsub.start(20)
    timerPose = qtgqt.QtCore.QTimer()
    timerPose.timeout.connect(ph.updatePose)
    timerPose.start(20)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
