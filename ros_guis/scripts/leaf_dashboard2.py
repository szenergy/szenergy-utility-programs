#!/usr/bin/env python
from __future__ import print_function
import rospy, rostopic 
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
        self.win.setWindowTitle("Leaf dashboard 2")
        self.win.resize(700, 600)
        self.win.setCentralWidget(area)
        dock1 = darea.Dock("Pose", size = (1,1))  # give this dock minimum possible size
        dock2 = darea.Dock("Lidar", size = (1,1))  # give this dock minimum possible size
        dock3 = darea.Dock("Camera", size = (1,1))  # give this dock minimum possible size
        dock4 = darea.Dock("dock N", size = (500,400)) # size is only a suggestion
        area.addDock(dock1, "left")
        area.addDock(dock2, "bottom", dock1)
        area.addDock(dock3, "bottom", dock2)
        area.addDock(dock4, "bottom", dock3)
        widg1 = pg.LayoutWidget()
        dhLabel = qtgqt.QtGui.QLabel("Duro:"); dhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dhLabel.setAlignment(pg.QtCore.Qt.AlignRight); dhLabel.setFixedSize(50, 25)
        dsLabel = qtgqt.QtGui.QLabel("Duro:"); dsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dsLabel.setAlignment(pg.QtCore.Qt.AlignRight); dsLabel.setFixedSize(50, 25)
        nhLabel = qtgqt.QtGui.QLabel("Nova:"); nhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        nsLabel = qtgqt.QtGui.QLabel("Nova:"); nsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nsLabel.setAlignment(pg.QtCore.Qt.AlignRight)     
        self.duroHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.duroRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.novaHzLabel = qtgqt.QtGui.QLabel("  **.* Hz")
        self.novaRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.pausePoseBtn = qtgqt.QtGui.QPushButton("Pause")
        self.savePoseBtn = qtgqt.QtGui.QPushButton("Save")
        self.speedLabel = qtgqt.QtGui.QLabel(" **.* Km/h")
        self.angleLabel = qtgqt.QtGui.QLabel(" **.* rad")
        widg1.addWidget(dhLabel, row=1, col=0)
        widg1.addWidget(dsLabel, row=1, col=2)
        widg1.addWidget(nhLabel, row=2, col=0)
        widg1.addWidget(nsLabel, row=2, col=2)
        widg1.addWidget(self.duroHzLabel, row=1, col=1)
        widg1.addWidget(self.duroRtkLabel, row=1, col=3)
        widg1.addWidget(self.novaHzLabel, row=2, col=1)
        widg1.addWidget(self.novaRtkLabel, row=2, col=3)
        widg1.addWidget(self.speedLabel, row=3, col=1)
        widg1.addWidget(self.angleLabel, row=3, col=3)
        widg1.addWidget(self.savePoseBtn, row=4, col=1)
        widg1.addWidget(self.pausePoseBtn, row=4, col=3)
        widg1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.novaHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.novaRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.duroHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(224, 166, 58)")
        self.duroRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(244, 166, 58)")
        self.angleLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.speedLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.pauseLidarBtn = qtgqt.QtGui.QPushButton("Pause")
        dock1.addWidget(widg1)
        self.state = None
        self.widg2 = pg.PlotWidget(title="Gps difference")
        self.widg2.setAspectLocked(True)
        self.pltGpsOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = blueB)
        self.pltLeafOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widg2.showGrid(x=True, y=True)
        self.widg2.addItem(self.pltGpsOdom)
        self.widg2.addItem(self.pltLeafOdom)
        dock4.addWidget(self.widg2)
        self.pauseCamBtn = qtgqt.QtGui.QPushButton("Pause")
        self.pausePoseBtn.clicked.connect(self.pausePose)
        self.pauseLidarBtn.clicked.connect(self.pauseLidar)
        self.pauseCamBtn.clicked.connect(self.pauseCamera)
        self.savePoseBtn.clicked.connect(self.saveToCsv)
        self.tGps = pg.TextItem(text = "Gps", color = blue)
        self.tLeaf = pg.TextItem(text = "Leaf odom", color = red)
        self.tstart = pg.TextItem(text = "Start", color = white)
        widg2 = pg.LayoutWidget()
        widg3 = pg.LayoutWidget()
        
        shLabel = qtgqt.QtGui.QLabel("Sick:"); shLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); shLabel.setAlignment(pg.QtCore.Qt.AlignRight); shLabel.setFixedSize(50, 25)
        olhLabel = qtgqt.QtGui.QLabel("OusterLeft:"); olhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); olhLabel.setAlignment(pg.QtCore.Qt.AlignRight); olhLabel.setFixedSize(80, 25)
        orhLabel = qtgqt.QtGui.QLabel("OusterRight:"); orhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); orhLabel.setAlignment(pg.QtCore.Qt.AlignRight); orhLabel.setFixedSize(95, 25)
        vlhLabel = qtgqt.QtGui.QLabel("VeloLeft:"); vlhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); vlhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        vrhLabel = qtgqt.QtGui.QLabel("VeloRight:"); vrhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); vrhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        self.sickHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.sickHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(224, 166, 58)")
        self.ousterLefHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.ousterLefHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.ousterRigHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.ousterRigHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(16, 200, 166)")
        self.veloLefHz = qtgqt.QtGui.QLabel(" **.* Hz")
        self.veloLefHz.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")
        self.veloRigHz = qtgqt.QtGui.QLabel(" **.* Hz")
        self.veloRigHz.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 66, 66)")        
        widg2.addWidget(shLabel, row=1, col=0)
        widg2.addWidget(olhLabel, row=1, col=2)
        widg2.addWidget(orhLabel, row=1, col=4)
        widg2.addWidget(vlhLabel, row=2, col=2)
        widg2.addWidget(vrhLabel, row=2, col=4)
        widg2.addWidget(self.sickHzLabel, row=1, col=1)
        widg2.addWidget(self.ousterLefHzLabel, row=1, col=3)
        widg2.addWidget(self.ousterRigHzLabel, row=1, col=5)
        widg2.addWidget(self.veloLefHz, row=2, col=3)
        widg2.addWidget(self.veloRigHz, row=2, col=5)
        widg2.addWidget(self.pauseLidarBtn, row=3, col=5)

        dock2.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock3.setStyleSheet("background-color: rgb(18, 20, 23);")
        widg2.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        widg3.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        zhLabel = qtgqt.QtGui.QLabel("Zed:"); zhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); zhLabel.setAlignment(pg.QtCore.Qt.AlignRight); zhLabel.setFixedSize(50, 25)
        zrLabel = qtgqt.QtGui.QLabel("Resolution:"); zrLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); zrLabel.setAlignment(pg.QtCore.Qt.AlignRight); zrLabel.setFixedSize(80, 25)
        self.zedHzLabel = qtgqt.QtGui.QLabel(" **.* Hz")
        self.zedHzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(123, 64, 133)")
        self.zedRzLabel = qtgqt.QtGui.QLabel("100x100 px")
        self.zedRzLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(123, 64, 133)")
        widg3.addWidget(zhLabel, row=1, col=0)
        widg3.addWidget(zrLabel, row=1, col=2)
        widg3.addWidget(self.zedHzLabel, row=1, col=1)
        widg3.addWidget(self.zedRzLabel, row=1, col=3)
        widg3.addWidget(self.pauseCamBtn, row=2, col=3)
        dock2.addWidget(widg2)
        dock3.addWidget(widg3)
        self.drawCircle(self.widg2)
        self.win.show()

    def updatePose(self):
        if self.paused == False:
            if self.leaf.duro_hz is not None:
                self.duroHzLabel.setText("%5.1f Hz" % (self.leaf.duro_hz))
            if self.leaf.nova_hz is not None:
                self.novaHzLabel.setText("%5.1f Hz" % (self.leaf.nova_hz))
            self.duroRtkLabel.setText(self.leaf.duro_rtk)
            self.novaRtkLabel.setText(self.leaf.nova_rtk)
            self.speedLabel.setText("%5.1f Km/h" % (self.leaf.leaf_speed * 3.6))
            self.angleLabel.setText("%5.3f rad" % (self.leaf.leaf_angl))
        if self.leaf.ouster_lef_hz is not None:
            self.ousterLefHzLabel.setText("%5.1f Hz" % (self.leaf.ouster_lef_hz))
        if self.leaf.ouster_rig_hz is not None:
            self.ousterRigHzLabel.setText("%5.1f Hz" % (self.leaf.ouster_rig_hz))    
        if self.leaf.nova_hz is not None:
            self.sickHzLabel.setText("%5.1f Hz" % (self.leaf.sick_hz))   
        if self.leaf.velo_lef_hz is not None:
            self.veloLefHz.setText("%5.1f Hz" % (self.leaf.velo_lef_hz))   
        if self.leaf.velo_rig_hz is not None:
            self.veloRigHz.setText("%5.1f Hz" % (self.leaf.velo_rig_hz))    
        if self.leaf.zed_hz is not None:
            self.zedHzLabel.setText("%5.1f Hz" % (self.leaf.zed_hz)) 
        if self.leaf.zed_info is not None:
            self.zedRzLabel.setText(self.leaf.zed_info)  
        self.pltGpsOdom.setPoints(self.leaf.pose_diff.x, self.leaf.pose_diff.y)    
        """
            self.pltGpsOdom.addPoints(self.leaf.gps_x, self.leaf.gps_y)
            self.pltLeafOdom.addPoints(self.leaf.leaf_x, self.leaf.leaf_y)
            if self.first_run == True:
                self.widg2.addItem(self.tGps)
                self.widg2.addItem(self.tLeaf)
                self.widg2.addItem(self.tstart)
                self.tstart.setPos(self.leaf.gps_x, self.leaf.gps_y)
                self.first_run = False
        """

    def updateLidar(self):
        None

    def updateCamera(self):
        None    

    def pausePose(self):
        self.paused = not self.paused
        self.leaf.sub_pos = not self.leaf.sub_pos
        """
        self.pltGpsOdom.data = self.pltGpsOdom.data[-1:-20:-1]
        self.pltLeafOdom.data = self.pltLeafOdom.data[-1:-20:-1]
        self.first_run = True
        """

    def pauseLidar(self):
        self.leaf.sub_lid = not self.leaf.sub_lid

    def pauseCamera(self):
        self.leaf.sub_cam = not self.leaf.sub_cam

    def drawCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 8, pen = pg.mkPen(None), brush = pg.mkBrush(80, 80, 80, 200))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.1)) * 1.7
        y = np.cos(np.arange(0, np.pi*2, 0.1)) * 1.7
        circle.addPoints(x, y)

    def saveToCsv(self):
        #print(self.leaf.pose_duro.x)
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
    duro_hz = -1.0; duro_rtk = ""
    nova_hz = -1.0; nova_rtk = ""
    ouster_lef_hz = -1.0; ouster_rig_hz = -1.0; sick_hz = -1.0
    velo_lef_hz = -1.0; velo_rig_hz = -1.0; zed_hz = -1.0; zed_info = "000x000 px"
    sub_pos = True; sub_pos_pre = True
    sub_lid = True; sub_lid_pre = True
    sub_cam = True; sub_cam_pre = True
    leaf_speed = 0.0; leaf_angl = 0.0
    pose_diff = Point2D(); pose_duro = Point2D(); pose_nova = Point2D()


    def __init__(self):
        self.registerPose()
        self.registerLidar()
        self.registerCamera()
    
    def registerPose(self):
        self.rp = rostopic.ROSTopicHz(window_size = 20)
        self.p0 = rospy.Subscriber("/gps/nova/current_pose", geomsg.PoseStamped, self.rp.callback_hz, callback_args="/gps/nova/current_pose")
        self.p1 = rospy.Subscriber("/gps/duro/current_pose", geomsg.PoseStamped, self.rp.callback_hz, callback_args="/gps/duro/current_pose")
        self.p2 = rospy.Subscriber("/gps/duro/current_pose", geomsg.PoseStamped, self.duroPoseCallBack)
        self.p3 = rospy.Subscriber("/gps/nova/current_pose", geomsg.PoseStamped, self.novaPoseCallBack)
        self.p4 = rospy.Subscriber("/gps/duro/status_string", rosmsg.String, self.duroRtkStatusCallBack)
        self.p5 = rospy.Subscriber("/gps/nova/bestvel", novamsg.NovatelVelocity, self.novaRtkStatusCallback)
        self.p6 = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus, self.vehicleStatusCallback)
    
    def unregisterPose(self):
        self.p0.unregister()
        self.p1.unregister()
        self.p2.unregister()
        self.p3.unregister()
        self.p4.unregister()
        self.p5.unregister()

    def registerLidar(self):
        self.rl = rostopic.ROSTopicHz(window_size = 20)
        self.l0 = rospy.Subscriber("/left_os1/os1_cloud_node/points", senmsg.PointCloud2, self.rl.callback_hz, callback_args="/left_os1/os1_cloud_node/points")
        self.l1 = rospy.Subscriber("/right_os1/os1_cloud_node/points", senmsg.PointCloud2, self.rl.callback_hz, callback_args="/right_os1/os1_cloud_node/points")
        self.l2 = rospy.Subscriber("/cloud", senmsg.PointCloud2, self.rl.callback_hz, callback_args="/cloud")
        self.l3 = rospy.Subscriber("/velodyne_left/velodyne_points", senmsg.PointCloud2, self.rl.callback_hz, callback_args="/velodyne_left/velodyne_points")
        self.l4 = rospy.Subscriber("/velodyne_right/velodyne_points", senmsg.PointCloud2, self.rl.callback_hz, callback_args="/velodyne_right/velodyne_points")
    
    def unregisterLidar(self):
        self.l0.unregister()
        self.l1.unregister()
        self.l2.unregister()
        self.l3.unregister()
        self.l4.unregister()

    def registerCamera(self):
        self.rc = rostopic.ROSTopicHz(window_size = 20)
        self.c0 = rospy.Subscriber("/zed_node/stereo/image_rect_color", senmsg.Image, self.rc.callback_hz, callback_args="/zed_node/stereo/image_rect_color")
        self.c1 = rospy.Subscriber("/zed_node/stereo/image_rect_color", senmsg.Image, self.zedCallBack)
   
    def unregisterCamera(self):
        self.c0.unregister()
        self.c1.unregister()

    def zedCallBack(self, msg):
        self.zed_info = str(msg.width) + "x" + str(msg.height) + " px"

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

    def updateHz(self):
        if self.sub_pos:
            try:
                self.duro_hz, _, _, _, _ = self.rp.get_hz("/gps/duro/current_pose")
            except (TypeError, AttributeError):
                self.duro_hz = 0.0
            try:
                self.nova_hz, _, _, _, _ = self.rp.get_hz("/gps/nova/current_pose")
            except (TypeError, AttributeError):
                self.nova_hz = 0.0
        if self.sub_lid:
            try:
                self.ouster_lef_hz, _, _, _, _ = self.rl.get_hz("/left_os1/os1_cloud_node/points")
            except (TypeError, AttributeError):
                self.ouster_lef_hz = 0.0
            try:
                self.ouster_rig_hz, _, _, _, _ = self.rl.get_hz("/right_os1/os1_cloud_node/points")
            except (TypeError, AttributeError):
                self.ouster_rig_hz = 0.0
            try:
                self.sick_hz, _, _, _, _ = self.rl.get_hz("/cloud")
            except (TypeError, AttributeError):
                self.sick_hz = 0.0
            try:
                self.velo_lef_hz, _, _, _, _ = self.rl.get_hz("/velodyne_left/velodyne_points")
            except (TypeError, AttributeError):
                self.velo_lef_hz = 0.0
            try:
                self.velo_rig_hz, _, _, _, _ = self.rl.get_hz("/velodyne_right/velodyne_points")
            except (TypeError, AttributeError):
                self.velo_rig_hz = 0.0
        if self.sub_cam:
            try:
                self.zed_hz, _, _, _, _ = self.rc.get_hz("/zed_node/stereo/image_rect_color")
            except (TypeError, AttributeError):
                self.zed_hz = 0.0

    def handleRegistering(self):      
        if self.sub_pos != self.sub_pos_pre: # if subscribe / unsubscribe
            print("Subscribed to pose topics: ", self.sub_pos)
            self.sub_pos_pre = self.sub_pos
            if self.sub_pos == True:
                self.registerPose()
            elif self.sub_pos == False:
                self.unregisterPose()
        if self.sub_lid != self.sub_lid_pre: # if subscribe / unsubscribe
            print("Subscribed to LIDAR topics: ", self.sub_lid)
            self.sub_lid_pre = self.sub_lid
            if self.sub_lid == True:
                self.registerLidar()
            elif self.sub_lid == False:
                self.unregisterLidar()
        if self.sub_cam != self.sub_cam_pre: # if subscribe / unsubscribe
            print("Subscribed to camera topics: ", self.sub_lid)
            self.sub_cam_pre = self.sub_cam
            if self.sub_cam == True:
                self.registerCamera()
            elif self.sub_lid == False:
                self.unregisterCamera()        
  

if __name__ == "__main__":
    import sys
    print(__file__, "- message reader started ... ")
    rospy.init_node("leafplotter", anonymous=True)
    leafSub = LeafSubscriber()
    ph = PlotHandler(leafSub)
    ph.initializePlot()
    timerHz = qtgqt.QtCore.QTimer()
    timerHz.timeout.connect(leafSub.updateHz)
    timerHz.start(200)
    timerSubUnsub = qtgqt.QtCore.QTimer()
    timerSubUnsub.timeout.connect(leafSub.handleRegistering)
    timerSubUnsub.start(20)
    timerPose = qtgqt.QtCore.QTimer()
    timerPose.timeout.connect(ph.updatePose)
    timerPose.start(30)
    timerLidar = qtgqt.QtCore.QTimer()
    timerLidar.timeout.connect(ph.updateLidar)
    timerLidar.start(30)
    timerCamera = qtgqt.QtCore.QTimer()
    timerCamera.timeout.connect(ph.updateCamera)
    timerCamera.start(30)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
