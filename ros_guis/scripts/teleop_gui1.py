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

class PlotHandler(object):
    def __init__(self, vehicle):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.vehicle = vehicle
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()
        self.rospack = rospkg.RosPack()

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
        self.win.resize(1200,600)
        self.win.setCentralWidget(self.area)

        self.dleft1 = darea.Dock("left 1", size = (1,1))  # give this dock minimum possible size
        self.dleft2 = darea.Dock("left 2", size = (500,400)) # size is only a suggestion
        self.dright1 = darea.Dock("right 1", size=(500,200))
        self.dright2 = darea.Dock("right 2", size=(500,200))
        self.area.addDock(self.dleft1, "left")
        self.area.addDock(self.dleft2, "bottom", self.dleft1)
        self.area.addDock(self.dright1, "right")
        self.area.addDock(self.dright2, "below", self.dright1)   ## place dright2 at top edge of dright1
        self.area.moveDock(self.dright2, "below", self.dright1)
        self.wleft1 = pg.LayoutWidget()
        self.accLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.posLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.saveBtn = qtgqt.QtGui.QPushButton("Save dock state")
        self.restoreBtn = qtgqt.QtGui.QPushButton("Restore dock state")
        self.clrBtn = qtgqt.QtGui.QPushButton("Clear")
        self.restoreBtn.setEnabled(False)
        self.wleft1.addWidget(self.accLabel, row=0, col=0)
        self.wleft1.addWidget(self.posLabel, row=0, col=1)
        self.wleft1.addWidget(self.clrBtn, row=0, col=2)
        self.wleft1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        self.dleft1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.accLabel.setStyleSheet("font-family: Monospace; font: 30pt; background-color: rgb(44, 48, 56)")
        self.dleft1.addWidget(self.wleft1)
        self.state = None
        self.wleft2 = pg.PlotWidget(title="Plot left 2 (bottom)")
        self.wleft2.setAspectLocked(True)
        self.plot_left2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(6,106,166,255))
        self.wleft2.showGrid(x=True, y=True)
        self.wleft2.addItem(self.plot_left2)
        self.dleft2.addWidget(self.wleft2)
        self.clrBtn.clicked.connect(self.clear)
        self.tcurr = pg.TextItem(text="Teleop", color = white)
        self.tstart = pg.TextItem(text="Start", color = red)

        self.wright1 = pg.PlotWidget(title="Plot right 1, random adat")
        self.wright1.plot(np.random.normal(size=20))
        self.wright1.showGrid(x=True, y=True)
        self.dright1.addWidget(self.wright1)
        self.wright2 = pg.PlotWidget(title="Plot right")
        self.plot_right2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(200,66,66,255))
        self.wright2.showGrid(x=True, y=True)
        self.wright2.addItem(self.plot_right2)
        self.dright2.addWidget(self.wright2)

        self.blueWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=10))
        self.wleft2.addItem(self.blueWheelHoriz)
        self.blueWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=10))
        self.wleft2.addItem(self.blueWheelVertic)
        self.blueWheelText1 = pg.TextItem(text="-", color = blue)
        self.wleft2.addItem(self.blueWheelText1)
        self.drawBlueCircle(self.wleft2)

        self.drawRedCircle(self.wleft2)
        self.redWheelHoriz = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=10))
        self.redWheelText1 = pg.TextItem(text="-", color = red)
        self.wleft2.addItem(self.redWheelHoriz)
        self.wleft2.addItem(self.redWheelText1)

        self.redWheelVertic = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=10))
        self.wleft2.addItem(self.redWheelVertic)


        self.wleft2.setAspectLocked(True)
        self.win.show()

    def updateFirstPlot(self):
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
        self.accLabel.setText("x: %9.6f\ny: %9.6f\nz: %9.6f" % (self.vehicle.imu_acc_x, self.vehicle.imu_acc_y, self.vehicle.imu_acc_z))        
        self.posLabel.setText("x: %9.6f\ny: %9.6f" % (self.vehicle.wheel_actual_rad, self.vehicle.odom_data_y))          

    def clear(self):
        self.plot_left2.data = self.plot_left2.data[-1:-20:-1]
        self.first_run = True
    
    def drawBlueCircle(self, to_plot):
        circle = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = pg.mkBrush(6, 106, 166))
        to_plot.addItem(circle)
        to_plot.setAspectLocked(lock = True, ratio = 1)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 80
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 80
        circle.addPoints(x, y)

    def drawRedCircle(self, to_plot):
        circleB = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = pg.mkBrush(200, 66, 66))
        to_plot.addItem(circleB)
        x = np.sin(np.arange(0, np.pi*2, 0.01)) * 100
        y = np.cos(np.arange(0, np.pi*2, 0.01)) * 100
        circleB.addPoints(x, y)


class TeleopSub(object):
    def __init__(self):
        #rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/wheel_angle_deg", stdmsg.Float32, self.wheelDegCallBack)
        rospy.Subscriber("/gps/nova/imu", senmsg.Imu, self.imuCallBack)


    def imuCallBack(self, msg):
        self.imu_acc_x = np.array([msg.linear_acceleration.x])
        self.imu_acc_y = np.array([msg.linear_acceleration.y])
        self.imu_acc_z = np.array([msg.linear_acceleration.z])
        #print("imu(xyz):  %8.4f %8.4f %8.4f" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

    def wheelDegCallBack(self, msg_deg): 
        self.wheel_actual_rad = np.deg2rad(np.array([msg_deg.data]))
        self.wheel_gamepa_rad = self.wheel_actual_rad +  (np.random.uniform(.2, 1.1) )
        self.odom_data_y = np.array([msg_deg.data])
        #print("odom: %.4f %.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))




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