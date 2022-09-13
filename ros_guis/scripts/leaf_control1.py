#!/usr/bin/env python
from __future__ import print_function
import rospy, rostopic, roslaunch, rospkg, tf
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import geometry_msgs.msg as geomsg
import sensor_msgs.msg as senmsg
try:
    import novatel_gps_msgs.msg as novamsg
except:
    None
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
        self.rospack = rospkg.RosPack()

    def initializePlot(self):
        self.first_run = True
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        darkyellow = (224, 166, 58); darkyellowB = pg.mkBrush(224, 166, 58, 200);
        self.win.setWindowTitle("Leaf control 1")
        self.win.setWindowIcon(qtgqt.QtGui.QIcon(self.rospack.get_path("ros_guis") + "/img/icon01.png"))
        self.win.resize(700, 600)
        self.win.setCentralWidget(area)
        dock1def = darea.Dock("Default", size = (1,1))  # give this dock minimum possible size
        dock2oth = darea.Dock("Others", size = (1,1))  # give this dock minimum possible size
        dock3ctr = darea.Dock("Control", size = (1,1))  # give this dock minimum possible size
        dock4gps = darea.Dock("2 Gps visualization", size = (500,400)) # size is only a suggestion
        area.addDock(dock1def, "left")
        area.addDock(dock2oth, "bottom", dock1def)
        area.addDock(dock3ctr, "above", dock2oth)
        area.addDock(dock4gps, "bottom", dock3ctr)
        dhLabel = qtgqt.QtGui.QLabel("Duro:"); dhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dhLabel.setAlignment(pg.QtCore.Qt.AlignRight); dhLabel.setFixedSize(50, 25)
        dsLabel = qtgqt.QtGui.QLabel("Duro:"); dsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); dsLabel.setAlignment(pg.QtCore.Qt.AlignRight); dsLabel.setFixedSize(50, 25)
        nhLabel = qtgqt.QtGui.QLabel("Nova:"); nhLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nhLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        nsLabel = qtgqt.QtGui.QLabel("Nova:"); nsLabel.setStyleSheet("background-color: rgb(4, 4, 4);"); nsLabel.setAlignment(pg.QtCore.Qt.AlignRight)     
        self.duroRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.novaRtkLabel = qtgqt.QtGui.QLabel("+++")
        self.pauseSensorReadClickedBtn = qtgqt.QtGui.QPushButton("Pause")
        self.savePoseBtn = qtgqt.QtGui.QPushButton("Save")
        self.allSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start AllSensor")
        self.tfSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start TF")
        self.tfDuroLaunchBtn = qtgqt.QtGui.QPushButton("Start TF Duro")
        self.tfNovaLaunchBtn = qtgqt.QtGui.QPushButton("Start TF Nova")
        self.duroSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Duro GPS")
        self.novaSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Nova GPS")
        self.zedSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start ZED camera")
        self.sickSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start SICK")
        self.ousterLeftSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start left Ouster")
        self.ousterRightSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start right Ouster")
        self.veloLeftSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start left Velodyne")
        self.veloRightSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start right Velodyne")
        self.canSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start CAN")
        self.radarSensorLaunchBtn = qtgqt.QtGui.QPushButton("Start Radar")
        self.mpcOnOffBtn = qtgqt.QtGui.QPushButton("Disable MPC")
        self.temporaryLaunchBtn = qtgqt.QtGui.QPushButton("Start temporary")
        self.loadWaypointBtn = qtgqt.QtGui.QPushButton("Load waypoints")
        self.saveWaypointBtn = qtgqt.QtGui.QPushButton("Save waypoints")
        self.carParamsBtn = qtgqt.QtGui.QPushButton("Publish car parameters")
        self.imuLaunchBtn = qtgqt.QtGui.QPushButton("Start IMU")
        self.odomLaunchBtn = qtgqt.QtGui.QPushButton("Start Odom")
        self.selectFileBtn = qtgqt.QtGui.QPushButton("...")
        self.selectFileBtn.setMaximumWidth(22)
        self.speedLabel = qtgqt.QtGui.QLabel(" **.* Km/h")
        self.isAutonomLabel = qtgqt.QtGui.QLabel("-")
        self.angleLabel = qtgqt.QtGui.QLabel(" **.* rad")
        self.csvLabel = qtgqt.QtGui.QLabel("none"); self.csvLabel.setAlignment(pg.QtCore.Qt.AlignRight)
        self.csvTextbox = qtgqt.QtGui.QLineEdit()
        self.allSensorLaunched = False
        self.waypointLoaded = False
        self.waypointSaving = False
        self.tfLaunched = False
        self.tfSensorLaunched = False
        self.tfDuroLaunched = False
        self.tfNovaLaunched = False
        self.duroSensorLaunched = False
        self.novaSensorLaunched = False
        self.zedSensorLaunched = False
        self.sickSensorLaunched = False
        self.ousterLeftSensorLaunched = False
        self.ousterRightSensorLaunched = False
        self.veloLeftSensorLaunched = False
        self.veloRightSensorLaunched = False
        self.canSensorLaunched = False
        self.radarSensorLaunched = False
        self.temporaryLaunched = False
        self.imuLaunched = False
        self.carParamsLaunched = False
        self.odomLaunched = False
        self.mpcFollow = True
        widg1def = pg.LayoutWidget()
        widg1def.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1def.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock1def.addWidget(widg1def)
        self.novaRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(6, 106, 166)")
        self.duroRtkLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(244, 166, 58)")
        self.angleLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.speedLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
        self.isAutonomLabel.setStyleSheet("font-family: Monospace; font: 20pt; color: rgb(200, 200, 200)")
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
        widg1def.addWidget(self.isAutonomLabel, row=4, col=4)
        widg1def.addWidget(self.pauseSensorReadClickedBtn, row=4, col=7)
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
        self.pltDuroOrientation = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(244, 166, 58), width=6))
        self.pltNovaOrientation = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=8))
        self.pltLeafOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widgGps.showGrid(x=True, y=True)
        self.widgGps.addItem(self.pltGpsOdom)
        self.widgGps.addItem(self.pltNovaOrientation)
        self.widgGps.addItem(self.pltDuroOrientation)
        self.widgGps.addItem(self.pltLeafOdom)
        dock4gps.addWidget(self.widgGps)
        self.pauseSensorReadClickedBtn.clicked.connect(self.pauseSensorReadClicked)
        self.savePoseBtn.clicked.connect(self.saveToCsvClicked)
        self.allSensorLaunchBtn.clicked.connect(self.startAllSensorClicked)
        self.tfSensorLaunchBtn.clicked.connect(self.startTFClicked)
        self.tfDuroLaunchBtn.clicked.connect(self.tfDuroClicked)
        self.tfNovaLaunchBtn.clicked.connect(self.tfNovaClicked)
        self.zedSensorLaunchBtn.clicked.connect(self.zedSensorClicked)
        self.duroSensorLaunchBtn.clicked.connect(self.duroSensorClicked)
        self.novaSensorLaunchBtn.clicked.connect(self.novaSensorClicked)
        self.sickSensorLaunchBtn.clicked.connect(self.sickSensorClicked)
        self.ousterLeftSensorLaunchBtn.clicked.connect(self.ousterLeftSensorClicked)
        self.ousterRightSensorLaunchBtn.clicked.connect(self.ousterRightSensorClicked)
        self.veloLeftSensorLaunchBtn.clicked.connect(self.veloLeftSensorClicked)
        self.veloRightSensorLaunchBtn.clicked.connect(self.veloRightSensorClicked)        
        self.canSensorLaunchBtn.clicked.connect(self.canSensorClicked)
        self.radarSensorLaunchBtn.clicked.connect(self.radarSensorClicked)
        self.mpcOnOffBtn.clicked.connect(self.mpcOnOffBtnClicked)
        self.temporaryLaunchBtn.clicked.connect(self.temporaryClicked)
        self.loadWaypointBtn.clicked.connect(self.loadCsvClicked)
        self.saveWaypointBtn.clicked.connect(self.saveCsvClicked)
        self.selectFileBtn.clicked.connect(self.selectCsvClicked)
        self.carParamsBtn.clicked.connect(self.carParamsClicked)   
        self.imuLaunchBtn.clicked.connect(self.startImuClicked) 
        self.odomLaunchBtn.clicked.connect(self.startOdomClicked)    
        
        self.tGps = pg.TextItem(text = "Gps", color = blue)
        self.tLeaf = pg.TextItem(text = "Leaf odom", color = red)
        self.tstart = pg.TextItem(text = "Start", color = white)

        
        dock2oth.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.drawCircle(self.widgGps)
        # other controls
        widg2oth = pg.LayoutWidget()
        dock2oth.setStyleSheet("background-color: rgb(18, 20, 23);")
        widg2oth.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        widg2oth.addWidget(self.tfSensorLaunchBtn, row=1, col=1)
        widg2oth.addWidget(self.tfDuroLaunchBtn, row = 1, col = 2)
        widg2oth.addWidget(self.tfNovaLaunchBtn, row = 1, col = 3)
        widg2oth.addWidget(self.zedSensorLaunchBtn, row = 2, col = 1)
        widg2oth.addWidget(self.duroSensorLaunchBtn, row = 2, col = 2)
        widg2oth.addWidget(self.novaSensorLaunchBtn, row = 2, col = 3)
        widg2oth.addWidget(self.sickSensorLaunchBtn, row = 3, col = 1)
        widg2oth.addWidget(self.ousterLeftSensorLaunchBtn, row = 3, col = 2)
        widg2oth.addWidget(self.ousterRightSensorLaunchBtn, row = 3, col = 3)
        widg2oth.addWidget(self.canSensorLaunchBtn, row = 4, col = 1)
        widg2oth.addWidget(self.radarSensorLaunchBtn, row = 5, col = 1)
        widg2oth.addWidget(self.mpcOnOffBtn, row = 5, col = 3)
        widg2oth.addWidget(self.veloLeftSensorLaunchBtn, row = 4, col = 2)
        widg2oth.addWidget(self.veloRightSensorLaunchBtn, row = 4, col = 3)
        widg2oth.addWidget(self.temporaryLaunchBtn, row = 5, col = 2)
        widg2oth.addWidget(self.carParamsBtn, row = 6, col = 1)
        widg2oth.addWidget(self.imuLaunchBtn, row = 6, col = 2)
        widg2oth.addWidget(self.odomLaunchBtn, row = 6, col = 3)

        dock2oth.addWidget(widg2oth)

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
        dock3ctr.addWidget(widg3ctr)
        widg3ctr.addWidget(self.allSensorLaunchBtn, row=1, col=1)
        try:
            current_file = rospy.get_param("waypoint_file_name")
            self.csvLabel.setText(os.path.basename(str(current_file)))
        except:
            self.csvLabel.setText("No waypoint_file_name param")
        self.pauseSensorReadClicked() # start paused - the same effect as pushed the pause button
        self.win.show()

    def updatePose(self):
        if self.leaf.stop_slow == False:
            self.duroRtkLabel.setText(self.leaf.duro_rtk)
            self.novaRtkLabel.setText(self.leaf.nova_rtk)
            self.speedLabel.setText("%5.1f Km/h" % (self.leaf.leaf_speed * 3.6))
            self.angleLabel.setText("%5.3f rad" % (self.leaf.leaf_angl))
            self.isAutonomLabel.setText(str(self.leaf.leaf_is_autonomous))
            self.ousterLefLabel.setText(self.leaf.ouster_lef_ok)
            self.ousterRigLabel.setText(self.leaf.ouster_rig_ok)
            self.sickOkLabel.setText(self.leaf.sick_ok)
            self.zedOkLabel.setText(self.leaf.zed_ok)
            self.veloLef.setText(self.leaf.velo_lef_ok)
            self.veloRig.setText(self.leaf.velo_rig_ok)
            self.pltGpsOdom.setPoints(self.leaf.pose_diff.x, self.leaf.pose_diff.y)
            self.pltDuroOrientation.setData(np.array([np.cos(self.leaf.duro_orientation), 0.0]), np.array([np.sin(self.leaf.duro_orientation), 0.0]))
            self.pltNovaOrientation.setData(np.array([np.cos(self.leaf.nova_orientation) * 1.2, 0.0]), np.array([np.sin(self.leaf.nova_orientation)  * 1.2, 0.0]))

    def tfDuroClicked(self): 
        if self.tfNovaLaunched is True:
            rospy.logerr("Novatel TF publisher is active, it will be shut down and replaced by Duro (leaf_control1)")
            self.tfNovaClicked() # shut down novatel tf publisher
        if self.tfDuroLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_duro_global_frame_tf_publisher.statictf.launch")
            self.launchDtf = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchDtf.start()
            rospy.loginfo(launchStr + " started")
            self.tfDuroLaunchBtn.setText("Stop tfDuro")
            self.tfDuroLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchDtf.shutdown()
            rospy.loginfo("tfDuro stopped.....")
            self.tfDuroLaunchBtn.setText("Start tfDuro")
            self.tfDuroLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.tfDuroLaunched = not self.tfDuroLaunched

    def tfNovaClicked(self): 
        if self.tfDuroLaunched is True:
            rospy.logerr("Duro TF publisher is active, it will be shut down and replaced by Novatel (leaf_control1)")
            self.tfDuroClicked() # shut duro tf publisher
        if self.tfNovaLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_novatel_global_frame_tf_publisher.statictf.launch")
            self.launchNtf = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchNtf.start()
            rospy.loginfo(launchStr + " started")
            self.tfNovaLaunchBtn.setText("Stop tfNova")
            self.tfNovaLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchNtf.shutdown()
            rospy.loginfo(" stopped.....")
            self.tfNovaLaunchBtn.setText("Start tfNova")
            self.tfNovaLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.tfNovaLaunched = not self.tfNovaLaunched

    def zedSensorClicked(self): 
        if self.zedSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/zed_camera_no_tf.launch")
            self.launchZED = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchZED.start()
            rospy.loginfo(launchStr + " started")
            self.zedSensorLaunchBtn.setText("Stop ZED")
            self.zedSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchZED.shutdown()
            rospy.loginfo("ZED stopped.....")
            self.zedSensorLaunchBtn.setText("Start ZED")
            self.zedSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.zedSensorLaunched = not self.zedSensorLaunched

    def duroSensorClicked(self): 
        if self.duroSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/gps.duro.launch")
            self.launchDuS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchDuS.start()
            rospy.loginfo(launchStr + " started")
            self.duroSensorLaunchBtn.setText("Stop duro")
            self.duroSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchDuS.shutdown()
            rospy.loginfo("Duro stopped.....")
            self.duroSensorLaunchBtn.setText("Start Duro sensor")
            self.duroSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.duroSensorLaunched = not self.duroSensorLaunched

    def temporaryClicked(self): 
        if self.temporaryLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("ros_guis"), "launch/temporary.launch")
            self.launchtemp = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchtemp.start()
            rospy.loginfo(launchStr + " started")
            self.temporaryLaunchBtn.setText("Stop temporary")
            self.temporaryLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchtemp.shutdown()
            rospy.loginfo("temporary stopped.....")
            self.temporaryLaunchBtn.setText("Start temporary")
            self.temporaryLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.temporaryLaunched = not self.temporaryLaunched

    def novaSensorClicked(self): 
        if self.novaSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/gps.nova.launch")
            self.launchNoS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchNoS.start()
            rospy.loginfo(launchStr + " started")
            self.novaSensorLaunchBtn.setStyleSheet("background-color: white")
            self.novaSensorLaunchBtn.setText("Stop Nova")
        else:
            self.launchNoS.shutdown()
            rospy.loginfo("Nova stopped.....")
            self.novaSensorLaunchBtn.setText("Start Nova sensor")
            self.novaSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.novaSensorLaunched = not self.novaSensorLaunched

    def sickSensorClicked(self): 
        if self.sickSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/sick_lms_111.nissan.launch")
            self.launchSick = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchSick.start()
            rospy.loginfo(launchStr + " started")
            self.sickSensorLaunchBtn.setText("Stop SICK")
            self.sickSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchSick.shutdown()
            rospy.loginfo("SICK stopped.....")
            self.sickSensorLaunchBtn.setText("Start SICK")
            self.sickSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.sickSensorLaunched = not self.sickSensorLaunched

    def ousterLeftSensorClicked(self): 
        if self.ousterLeftSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/ouster_left.launch")
            self.launchOuLe = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchOuLe.start()
            rospy.loginfo(launchStr + " started")
            self.ousterLeftSensorLaunchBtn.setText("Stop ousterLeft")
            self.ousterLeftSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchOuLe.shutdown()
            rospy.loginfo("ousterLeft stopped.....")
            self.ousterLeftSensorLaunchBtn.setText("Start ousterLeft")
            self.ousterLeftSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.ousterLeftSensorLaunched = not self.ousterLeftSensorLaunched

    def ousterRightSensorClicked(self): 
        if self.ousterRightSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/ouster_right.launch")
            self.launchOuRi = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchOuRi.start()
            rospy.loginfo(launchStr + " started")
            self.ousterRightSensorLaunchBtn.setText("Stop ousterRight")
            self.ousterRightSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchOuRi.shutdown()
            rospy.loginfo("ousterRight stopped.....")
            self.ousterRightSensorLaunchBtn.setText("Start ousterRight")
            self.ousterRightSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.ousterRightSensorLaunched = not self.ousterRightSensorLaunched

    def veloLeftSensorClicked(self): 
        if self.veloLeftSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/velodyne_left.launch")
            self.launchVeLe = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchVeLe.start()
            rospy.loginfo(launchStr + " started")
            self.veloLeftSensorLaunchBtn.setText("Stop veloLeft")
            self.veloLeftSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchVeLe.shutdown()
            rospy.loginfo("veloLeft stopped.....")
            self.veloLeftSensorLaunchBtn.setText("Start veloLeft")
            self.veloLeftSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.veloLeftSensorLaunched = not self.veloLeftSensorLaunched

    def veloRightSensorClicked(self): 
        if self.veloRightSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/velodyne_right.launch")
            self.launchVeRi = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchVeRi.start()
            rospy.loginfo(launchStr + " started")
            self.veloRightSensorLaunchBtn.setText("Stop veloRight")
            self.veloRightSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchVeRi.shutdown()
            rospy.loginfo(" stopped.....")
            self.veloRightSensorLaunchBtn.setText("Start veloRight")
            self.veloRightSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.veloRightSensorLaunched = not self.veloRightSensorLaunched

    def canSensorClicked(self):
        if self.canSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("can_leaf_driver"), "launch/nissan_can_control.launch")
            self.launchCAN = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchCAN.start()
            rospy.loginfo(launchStr + " started")
            self.canSensorLaunchBtn.setText("Stop CAN")
            self.canSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchCAN.shutdown()
            rospy.loginfo("CAN stopped.....")
            self.canSensorLaunchBtn.setText("Start CAN")
            self.canSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.canSensorLaunched = not self.canSensorLaunched

    def radarSensorClicked(self):
        if self.radarSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/radar_continental.launch")
            self.launchCAN = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchCAN.start()
            rospy.loginfo(launchStr + " started")
            self.radarSensorLaunchBtn.setText("Stop Radar")
            self.radarSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchCAN.shutdown()
            rospy.loginfo("CAN stopped.....")
            self.radarSensorLaunchBtn.setText("Start Radar")
            self.radarSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.radarSensorLaunched = not self.radarSensorLaunched

    def mpcOnOffBtnClicked(self):
        if self.mpcFollow is False:
            self.mpcOnOffBtn.setText("Disable MPC")
            self.mpcOnOffBtn.setStyleSheet("background-color: rgb(16, 200, 166)")
        else:
            self.mpcOnOffBtn.setText("Enable MPC")
            self.mpcOnOffBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.mpcFollow = not self.mpcFollow
        print(self.mpcFollow)


    def startTFClicked(self):
        if self.tfLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/tf_setup/tf_nissanleaf_statictf.launch")
            self.launchTF = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchTF.start()
            rospy.loginfo(launchStr + " started")
            self.tfSensorLaunchBtn.setText("Stop TF")
            self.tfSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchTF.shutdown()
            rospy.loginfo("TF stopped.....")
            self.tfSensorLaunchBtn.setText("Start TF")
            self.tfSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.tfLaunched = not self.tfLaunched

    def startAllSensorClicked(self):
        if self.allSensorLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/nissan.leaf.bringup.2020.A.launch")
            self.launchAS = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchAS.start()
            rospy.loginfo(launchStr + " started")
            self.allSensorLaunchBtn.setText("Stop AllSensor")
            self.allSensorLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchAS.shutdown()
            rospy.loginfo("All sensor stopped.....")
            self.allSensorLaunchBtn.setText("Start AllSensor")
            self.allSensorLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.allSensorLaunched = not self.allSensorLaunched

    def loadCsvClicked(self):
        if self.waypointLoaded is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            if self.mpcFollow is False:
                launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/waypoint.no.mpc.launch")
            else:
                launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/demo.waypoint.follow.launch")
            self.launchLC = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchLC.start()
            rospy.loginfo(launchStr + " started")
            self.loadWaypointBtn.setText("UnLoad waypoints")
            self.loadWaypointBtn.setStyleSheet("color: rgb(224, 166, 58)")
        else:
            self.launchLC.shutdown()
            rospy.loginfo("waypoint unloaded")
            self.loadWaypointBtn.setText("Load waypoints")
            self.loadWaypointBtn.setStyleSheet("color: rgb(200, 200, 200)")
        self.waypointLoaded = not self.waypointLoaded

    def saveCsvClicked(self):
        if self.waypointSaving is False:
            if len(str(self.csvTextbox.text())) < 3:
                self.csvTextbox.setText("tmp001.csv")
            new_csv = "/mnt/storage_1tb/waypoint/" + str(self.csvTextbox.text())
            rospy.set_param("/waypoint_saver/save_filename", new_csv)
            rospy.loginfo(new_csv + " set as /waypoint_saver/save_filename rosparam")
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/waypoint.saver.launch")
            self.launchSC = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchSC.start()
            rospy.loginfo(launchStr + " started")
            rospy.loginfo("waypoint saving started")
            self.saveWaypointBtn.setText("Finish waypoints")
            self.saveWaypointBtn.setStyleSheet("color: rgb(224, 166, 58)")
        else:
            self.launchSC.shutdown()
            rospy.loginfo("waypoint saving finished")
            self.saveWaypointBtn.setText("Save waypoints")
            self.saveWaypointBtn.setStyleSheet("color: rgb(200, 200, 200)")
        self.waypointSaving = not self.waypointSaving    

    def selectCsvClicked(self):
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

    def pauseSensorReadClicked(self):
        self.leaf.stop_slow = not self.leaf.stop_slow
        self.leaf.sub_pos = not self.leaf.sub_pos
        if self.leaf.stop_slow:
            self.pauseSensorReadClickedBtn.setText("UnPause")
            self.ousterLefLabel.setText("paused")
            self.ousterRigLabel.setText("paused")
            self.duroRtkLabel.setText("paused")
            self.novaRtkLabel.setText("paused")
            self.veloLef.setText("paused")
            self.veloRig.setText("paused")
            self.sickOkLabel.setText("paused")
        else:
            self.pauseSensorReadClickedBtn.setText("Pause")

    def drawCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 8, pen = pg.mkPen(None), brush = pg.mkBrush(80, 80, 80, 200))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.1)) * 1.7
        y = np.cos(np.arange(0, np.pi*2, 0.1)) * 1.7
        circle.addPoints(x, y)

    def saveToCsvClicked(self):
        #rospy.loginfo(self.leaf.pose_duro.x)
        f = open("gps.txt", "a+")
        data_to_save = np.array([self.leaf.pose_duro.x, self.leaf.pose_duro.y, self.leaf.pose_nova.x, self.leaf.pose_nova.y])
        np.savetxt(f, data_to_save, fmt="%.8f", delimiter=",", newline=" ")
        f.write("\n")
        f.close()

    def startImuClicked(self):
        if self.imuLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("fsm_imu"), "launch/fsm9.launch")
            self.launchImu = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchImu.start()
            rospy.loginfo(launchStr + " started")
            self.imuLaunchBtn.setText("Stop Imu")
            self.imuLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchImu.shutdown()
            rospy.loginfo("Imu stopped.....")
            self.imuLaunchBtn.setText("Start Imu")
            self.imuLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.imuLaunched = not self.imuLaunched

    def carParamsClicked(self):
        if self.carParamsLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/car_parameters_leaf.launch")
            self.launchcarParams = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchcarParams.start()
            rospy.loginfo(launchStr + " started")
            self.carParamsBtn.setText("car params set")
            self.carParamsBtn.setStyleSheet("background-color: white")

    def startOdomClicked(self):
        if self.odomLaunched is False:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launchStr = os.path.join(self.rospack.get_path("nissan_bringup"), "launch/sensory/odom.launch")
            self.launchodom = roslaunch.parent.ROSLaunchParent(uuid, [launchStr])
            self.launchodom.start()
            rospy.loginfo(launchStr + " started")
            self.odomLaunchBtn.setText("Stop Odom")
            self.odomLaunchBtn.setStyleSheet("background-color: white")
        else:
            self.launchodom.shutdown()
            rospy.loginfo("Imu stopped.....")
            self.odomLaunchBtn.setText("Start Odom")
            self.odomLaunchBtn.setStyleSheet("background-color: rgb(40, 44, 52)")
        self.odomLaunched = not self.odomLaunched

        



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
        #self.registerPose()
        self.ouster_lef_ok = "-"
        self.ouster_rig_ok = "-"
        self.zed_ok = "-"
        self.velo_lef_ok = "-"
        self.velo_rig_ok = "-"
        self.sick_ok = "-"
        self.duro_rtk = "-"
        self.nova_rtk = "-"
        self.duro_orientation = 0.0
        self.nova_orientation = 0.0
        self.iterator = 0
        self.stop_slow = False # unpaused
        self.p2 = self.p3 = self.p4 = self.p5 = self.p6 = None
        self.leaf_is_autonomous = "|"

    def slowGetMsg(self):
        if self.iterator == 100: # exeecute only every 100th time
            self.iterator = 0
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
        else:
            self.iterator += 1

    def registerPose(self):
        self.stop_slow = False
        self.p2 = rospy.Subscriber("/gps/duro/current_pose", geomsg.PoseStamped, self.duroPoseCallBack)
        self.p3 = rospy.Subscriber("/gps/nova/current_pose", geomsg.PoseStamped, self.novaPoseCallBack)
        self.p4 = rospy.Subscriber("/gps/duro/status_string", rosmsg.String, self.duroRtkStatusCallBack)
        try:
            self.p5 = rospy.Subscriber("/gps/nova/bestvel", novamsg.NovatelVelocity, self.novaRtkStatusCallback)
        except:
            rospy.logwarn("no novatel_gps_msgs.msg custom messages built")
            self.nova_rtk = "NoNovaCustomMsg"
        self.p6 = rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus, self.vehicleStatusCallback)
    
    def unregisterPose(self):
        try:
            self.stop_slow = True
            self.p2.unregister()
            self.p3.unregister()
            self.p4.unregister()
            self.p5.unregister()
            self.p6.unregister()
        except:
            None # not registered (subscubed) yet
        
    def novaPoseCallBack(self, msg):
        self.pose_nova.x = msg.pose.position.x
        self.pose_nova.y = msg.pose.position.y
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.nova_orientation = euler[2] - np.pi 

    def duroPoseCallBack(self, msg):
        self.pose_duro.x = msg.pose.position.x
        self.pose_duro.y = msg.pose.position.y
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.duro_orientation = euler[2] - np.pi 
        self.pose_diff = self.pose_duro -self.pose_nova

    def vehicleStatusCallback(self, msg):
        self.leaf_angl = msg.angle
        self.leaf_speed = msg.speed
        if msg.drivemode == 0: 
            self.leaf_is_autonomous = "DRIVER"
        elif msg.drivemode == 1:
            self.leaf_is_autonomous = "AUTONOMOUS"
        else:
            self.leaf_is_autonomous = "UNDEF"

    def novaRtkStatusCallback(self, msg):
        self.nova_rtk = msg.velocity_type

    def duroRtkStatusCallBack(self, msg):
        self.duro_rtk = msg.data

    def handleRegistering(self):      
        if self.sub_pos != self.sub_pos_pre: # if subscribe / unsubscribe
            rospy.loginfo("Subscribed to pose topics: " + str(self.sub_pos))
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
    timerSlowSub.start(30)
    timerSubUnsub = qtgqt.QtCore.QTimer()
    timerSubUnsub.timeout.connect(leafSub.handleRegistering)
    timerSubUnsub.start(20)
    timerPose = qtgqt.QtCore.QTimer()
    timerPose.timeout.connect(ph.updatePose)
    timerPose.start(20)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
