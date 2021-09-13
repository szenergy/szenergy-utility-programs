#!/usr/bin/env python
from __future__ import print_function
import rospy, rospkg
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg
import std_msgs.msg as stdmsg
import numpy as np
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea
import autoware_msgs.msg as autowmsgs


class PlotHandler(object):
    def __init__(self, vehicle):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.vehicle = vehicle
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()
        self.rospack = rospkg.RosPack()
        self.win.move

    def initializePlot(self):
        self.first_run = True
        white = (200, 200, 200)
        lightgray = (171, 178, 191)
        darkgray = (30, 40, 50)
        dark1 = (40, 44, 52)
        dark2 = (44, 48, 56)
        red = (200, 66, 66); red1b = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blue1b = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); green1b = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellow1b = pg.mkBrush(244, 244, 160, 200)
             
        self.win.setWindowTitle("Teleop plotter")
        self.win.setWindowIcon(qtgqt.QtGui.QIcon(self.rospack.get_path("ros_guis") + "/img/icon02.png"))
        self.win.resize(1500,2160)
        self.win.move(0,0)
        self.win.setCentralWidget(self.area)

        self.dlefttop = darea.Dock("left top", size = (1,1))  # give this dock minimum possible size
        self.dleftbottom = darea.Dock("left bottom", size = (500,400)) # size is only a suggestion
        self.dlefttop.hideTitleBar() # TODO 1
        self.dleftbottom.hideTitleBar() # TODO 1
        #self.dright1 = darea.Dock("right 1", size=(500,200))
        #self.dright2 = darea.Dock("right 2", size=(500,200))
        self.area.addDock(self.dlefttop, "left")
        self.area.addDock(self.dleftbottom, "bottom", self.dlefttop)
        #self.area.addDock(self.dright1, "right")
        #self.area.addDock(self.dright2, "below", self.dright1)   ## place dright2 at top edge of dright1
        #self.area.moveDock(self.dright2, "below", self.dright1)
        self.wleft1 = pg.LayoutWidget()
        self.speedLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.isAutonomLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.saveBtn = qtgqt.QtGui.QPushButton("Save dock state")
        self.restoreBtn = qtgqt.QtGui.QPushButton("Restore dock state")
        self.clrBtn = qtgqt.QtGui.QPushButton("Clear")
        self.restoreBtn.setEnabled(False)
        self.wleft1.addWidget(self.isAutonomLabel, row=0, col=0)
        self.wleft1.addWidget(self.speedLabel, row=1, col=0)
        #self.wleft1.addWidget(self.clrBtn, row=2, col=0) # TODO 1
        self.wleft1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        self.dlefttop.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.speedLabel.setStyleSheet("font-family: Monospace; font: 120pt; background-color: rgb(0, 0, 0)")
        self.isAutonomLabel.setStyleSheet("font: 120pt; background-color: rgb(200, 66, 66); color: rgb(44, 48, 56)")
        self.isAutonomLabel.setAlignment(pg.QtCore.Qt.AlignCenter)
        self.dlefttop.addWidget(self.wleft1)
        self.state = None
        #self.wleftbottom = pg.PlotWidget(title="Plot left 2 (bottom)") # TODO 1
        self.wleftbottom = pg.PlotWidget() # TODO 1
        self.wleftbottom.setAspectLocked(True)
        self.plot_left2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(6,106,166,255))
        self.wleftbottom.showGrid(x=True, y=True)
        self.wleftbottom.addItem(self.plot_left2)
        self.dleftbottom.addWidget(self.wleftbottom)
        self.wleftbottom.hideAxis('bottom') # TODO 1
        self.wleftbottom.hideAxis('left') # TODO 1
        self.clrBtn.clicked.connect(self.clear)
        self.tcurr = pg.TextItem(text="Teleop", color = white)
        self.tstart = pg.TextItem(text="Start", color = red)
        """
        self.wright1 = pg.PlotWidget(title="Plot right 1, random adat")
        self.wright1.plot(np.random.normal(size=20))
        self.wright1.showGrid(x=True, y=True)
        self.dright1.addWidget(self.wright1)
        self.wright2 = pg.PlotWidget(title="Plot right")
        self.plot_right2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(200,66,66,255))
        self.wright2.showGrid(x=True, y=True)
        self.wright2.addItem(self.plot_right2)
        self.dright2.addWidget(self.wright2)
        """

        self.blueWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=40))
        self.wleftbottom.addItem(self.blueWheelHoriz)
        self.blueWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=40))
        self.wleftbottom.addItem(self.blueWheelVertic)
        self.blueWheelText1 = pg.TextItem(text="-", color = blue)
        self.wleftbottom.addItem(self.blueWheelText1)
        self.drawBlueCircle(self.wleftbottom)

        self.drawRedCircle(self.wleftbottom)
        self.redWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=40))
        self.redWheelText1 = pg.TextItem(text="-", color = red)
        self.wleftbottom.addItem(self.redWheelHoriz)
        self.wleftbottom.addItem(self.redWheelText1)

        self.redWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=40))
        self.wleftbottom.addItem(self.redWheelVertic)


        self.wleftbottom.setAspectLocked(True)
        self.win.show()

    def updateFirstPlot(self):
        self.isAutonomLabel.setText(str(self.vehicle.leaf_is_autonomous))
        if str(self.vehicle.leaf_is_autonomous) == "UNDEF":
            self.isAutonomLabel.setStyleSheet("font: 120pt; background-color: rgb(244, 244, 160); color: rgb(44, 48, 56)")
        elif str(self.vehicle.leaf_is_autonomous) == "IN CAR DRIVER":
            self.isAutonomLabel.setStyleSheet("font: 120pt; background-color: rgb(6, 106, 166); color: rgb(44, 48, 56)")
        elif str(self.vehicle.leaf_is_autonomous) == "YOU DRIVE":
            self.isAutonomLabel.setStyleSheet("font: 120pt; background-color: rgb(200, 66, 66); color: rgb(44, 48, 56)")
        else:
            self.isAutonomLabel.setStyleSheet("font: 120pt; background-color: rgb(200, 200, 200); color: rgb(44, 48, 56)")

        """
        try:

            self.plot_right2.addPoints(self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y)
            if self.first_run == True:
                self.wright2.addItem(self.tcurr)
                self.wright2.addItem(self.tstart)
                self.tstart.setPos(self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y)
                self.first_run = False
            self.tcurr.setPos(self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y)

            None
        except:
            None # todo
        """
        None

    def updateSecondPlot(self):
        self.redWheelText1.setText("%.0f" % np.rad2deg(self.vehicle.wheel_actual_rad))
        self.blueWheelText1.setText("%.0f" % np.rad2deg(self.vehicle.wheel_gamepa_rad))
        self.redWheelText1.setPos(110 * np.cos(self.vehicle.wheel_actual_rad), 110 * np.sin(self.vehicle.wheel_actual_rad))
        self.blueWheelText1.setPos(-120 * np.cos(self.vehicle.wheel_gamepa_rad), -120 * np.sin(self.vehicle.wheel_gamepa_rad))
        r1 = np.array([100 * np.cos(self.vehicle.wheel_actual_rad), -100 * np.cos(self.vehicle.wheel_actual_rad)], dtype = np.float).flatten()
        r2 = np.array([100 * np.sin(self.vehicle.wheel_actual_rad), -100 * np.sin(self.vehicle.wheel_actual_rad)], dtype = np.float).flatten()
        r0 = np.array([100 * np.cos(self.vehicle.wheel_actual_rad - np.math.pi / 2), 0.], dtype = np.float)
        r3 = np.array([100 * np.sin(self.vehicle.wheel_actual_rad - np.math.pi / 2), 0.], dtype = np.float)
        self.redWheelHoriz.setData(r1, r2)
        self.redWheelVertic.setData(r0, r3)
        b1 = np.array([80 * np.cos(self.vehicle.wheel_gamepa_rad), -80 * np.cos(self.vehicle.wheel_gamepa_rad)], dtype = np.float).flatten()
        b2 = np.array([80 * np.sin(self.vehicle.wheel_gamepa_rad), -80 * np.sin(self.vehicle.wheel_gamepa_rad)], dtype = np.float).flatten()
        b0 = np.array([80 * np.cos(self.vehicle.wheel_gamepa_rad - np.math.pi / 2), 0.], dtype = np.float)
        b3 = np.array([80 * np.sin(self.vehicle.wheel_gamepa_rad - np.math.pi / 2), 0.], dtype = np.float)
        self.blueWheelHoriz.setData(b1, b2)
        self.blueWheelVertic.setData(b0, b3)


    def updateLabels(self):
        self.speedLabel.setText("actual: %4.1f km/h\nrefer : %4.1f km/h" % (self.vehicle.actual_speed, self.vehicle.ref_speed))        
        #self.isAutonomLabel.setText("x: %9.6f\ny: %9.6f" % (self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y))          

    def clear(self):
        self.plot_left2.data = self.plot_left2.data[-1:-20:-1]
        self.first_run = True
    
    def drawBlueCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 40, pen = pg.mkPen(None), brush = pg.mkBrush(6, 106, 166))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 80
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 80
        circle.addPoints(x, y)

    def drawRedCircle(self, to_plot):
        circleB = pg.ScatterPlotItem(size = 40, pen = pg.mkPen(None), brush = pg.mkBrush(200, 66, 66))
        to_plot.addItem(circleB)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 100
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 100
        circleB.addPoints(x, y)



class TeleopSub(object):
    def __init__(self):
        #rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/wheel_angle_deg", stdmsg.Float32, self.wheelDegCallBack)
        rospy.Subscriber("/vehicle_speed_kmph", stdmsg.Float32, self.speedKmpHCallBack)
        #rospy.Subscriber("/gps/nova/imu", senmsg.Imu, self.imuCallBack)
        rospy.Subscriber("/vehicle_status", autowmsgs.VehicleStatus, self.vehicleStatusCallback)
        rospy.Subscriber("/ctrl_cmd", autowmsgs.ControlCommandStamped, self.vehicleCtrlCallback)
        self.actual_speed = -1
        self.ref_speed = -1
        self.wheel_actual_rad = -0.01
        self.wheel_gamepa_rad =  0.01
        self.leaf_is_autonomous = "UNDEF"

    def imuCallBack(self, msg):
        self.imu_acc_x = np.array([msg.linear_acceleration.x])
        self.imu_acc_y = np.array([msg.linear_acceleration.y])
        self.imu_acc_z = np.array([msg.linear_acceleration.z])
        #print("imu(xyz):  %8.4f %8.4f %8.4f" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

    def speedKmpHCallBack(self, msg_kmph):
        self.actual_speed = np.array([msg_kmph.data])
        #self.ref_speed = self.actual_speed +  (np.random.uniform(.5, 4.1) ) # TODO

    def wheelDegCallBack(self, msg_deg): 
        self.wheel_actual_rad = np.deg2rad(np.array([msg_deg.data])) * 19.68 # 19.68 wheel to steering ratio
        #self.wheel_gamepa_rad = self.wheel_actual_rad +  (np.random.uniform(.2, 0.5) ) # TODO
        self.odom_data_y = np.array([msg_deg.data])
        #print("odom: %.4f %.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))

    def vehicleStatusCallback(self, msg):
        self.leaf_angl = msg.angle
        self.leaf_speed = msg.speed
        if msg.drivemode == 0: 
            self.leaf_is_autonomous = "IN CAR DRIVER"
        elif msg.drivemode == 1:
            self.leaf_is_autonomous = "YOU DRIVE"
        else:
            self.leaf_is_autonomous = "UNDEF"

    def vehicleCtrlCallback(self, msg_ctrl):
        self.ref_speed = msg_ctrl.cmd.linear_velocity 
        self.wheel_gamepa_rad = msg_ctrl.cmd.steering_angle * 19.68 # 19.68 wheel to steering ratio
        #print(self.ref_speed, self.wheel_gamepa_rad)

if __name__ == "__main__":
    import sys
    rospy.init_node("plotter_", anonymous=True)
    rospy.loginfo("Teleop GUI started ... ")
    vehSub = TeleopSub()
    ph = PlotHandler(vehSub)
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateSecondPlot)
    timer1.start(30)
    timer2 = qtgqt.QtCore.QTimer()
    timer2.timeout.connect(ph.updateFirstPlot)
    timer2.start(50)
    timer3 = qtgqt.QtCore.QTimer()
    timer3.timeout.connect(ph.updateLabels)
    timer3.start(30)


    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()