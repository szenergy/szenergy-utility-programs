#!/usr/bin/env python

"""
Waypoint and map editor
"""

import subprocess
import os
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea
#import rospy, rospkg, roslaunch
import numpy as np
import pandas as pd 

class PlotHandler():
    def __init__(self):
        pg.setConfigOptions(antialias=True)
        self.app = qtgqt.QtGui.QApplication([])

    def initializePlot(self):
        self.first_run = True
        self.paused = False
        self.launched = False
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        self.win.setWindowTitle("Waypoint and map editor")
        self.win.resize(1000, 800)
        self.win.setCentralWidget(area)
        dock1 = darea.Dock("dock 1", size = (1,1))  # give this dock minimum possible size
        dock2 = darea.Dock("dock 2", size = (500,400)) # size is only a suggestion
        area.addDock(dock1, "left")
        area.addDock(dock2, "bottom", dock1)
        widg1 = pg.LayoutWidget()
        self.csv1Label = qtgqt.QtGui.QLabel("none"); self.csv1Label.setAlignment(pg.QtCore.Qt.AlignRight)
        self.csv2Label = qtgqt.QtGui.QLabel("none"); self.csv2Label.setAlignment(pg.QtCore.Qt.AlignRight)
        self.selectFile1Btn = qtgqt.QtGui.QPushButton("Select file")
        self.selectFile2Btn = qtgqt.QtGui.QPushButton("Select file")
        self.selectFile1Btn.setMaximumWidth(100)
        self.selectFile2Btn.setMaximumWidth(100)
        widg1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.csv1Label.setStyleSheet("font: 10pt; color: rgb(6, 106, 166)") # blue
        self.csv2Label.setStyleSheet("font: 10pt; color: rgb(200, 66, 66)") # red
        widg1.addWidget(self.selectFile1Btn, row=1, col=2)
        widg1.addWidget(self.selectFile2Btn, row=2, col=2)
        widg1.addWidget(self.csv1Label, row=1, col=1)
        widg1.addWidget(self.csv2Label, row=2, col=1)
        dock1.addWidget(widg1)
        self.state = None
        self.widg2 = pg.PlotWidget(title="widg2 (bottom)")
        self.widg2.setAspectLocked(True)
        self.pltBlue = ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = blueB) # instead of pg.ScatterPlotItem use the override ScatterPlotItem
        self.pltRed = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widg2.showGrid(x=True, y=True)
        self.widg2.addItem(self.pltBlue)
        self.widg2.addItem(self.pltRed)
        dock2.addWidget(self.widg2)
        self.selectFile1Btn.clicked.connect(self.selectCsv1Clicked)
        self.selectFile2Btn.clicked.connect(self.selectCsv2Clicked)
        self.textSpeedArray = np.empty(1, dtype=object)
        self.win.show()

    def updateFirstPlot(self):     
        None

    def selectCsv1Clicked(self):
        dlg = qtgqt.QtGui.QFileDialog()
        dlg.setFileMode(qtgqt.QtGui.QFileDialog.AnyFile)
        dlg.setFilter("Text files (*.csv)")
        #dlg.setDirectory("/mnt/c")
        filenames = qtgqt.QtCore.QStringList()
        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.csv1Label.setText(os.path.basename(str(filenames[0])))          
            data = pd.read_csv(str(filenames[0])) 
            self.pltBlue.setPoints(np.asarray(data.x), np.asarray(data.y))
            self.textSpeedArray = np.empty(len(np.asarray(data.x)), dtype=object)
            #"""
            for i in range(len(np.asarray(data.x))):
                self.textSpeedArray[i] = pg.TextItem(text = "", color = (200, 200, 200))
                self.textSpeedArray[i].setText("%.1f" % data.velocity[i])
                self.textSpeedArray[i].setPos(np.asarray(data.x)[i], np.asarray(data.y)[i])
                self.widg2.addItem(self.textSpeedArray[i])
            #"""


    def selectCsv2Clicked(self):
        dlg = qtgqt.QtGui.QFileDialog()
        dlg.setFileMode(qtgqt.QtGui.QFileDialog.AnyFile)
        dlg.setFilter("Text files (*.csv)")
        #dlg.setDirectory("/mnt/c")
        filenames = qtgqt.QtCore.QStringList()
        if dlg.exec_():
            filenames = dlg.selectedFiles()
            self.csv2Label.setText(str(filenames[0])) #os.path.basename(str(filenames[0]))          
            data = pd.read_csv(str(filenames[0])) 
            self.pltRed.setPoints(np.asarray(data.x), np.asarray(data.y))
            self.textSpeedArray = np.empty(len(np.asarray(data.x)), dtype=object)

class ScatterPlotItem(pg.ScatterPlotItem):
    # override the default mousePressEvent
    def mousePressEvent(self, ev):
        if ev.button() == qtgqt.QtCore.Qt.LeftButton:
            pts = self.pointsAt(ev.pos())
            if len(pts) > 0:
                self.ptsClicked = pts
                ev.accept()
                self.sigClicked.emit(self, self.ptsClicked)
            else:
                ev.ignore()
        elif ev.button() == qtgqt.QtCore.Qt.RightButton:
            x = np.asarray([ev.pos().x()])
            y = np.asarray([ev.pos().y()])
            #print("Pressed: %.2f %.2f" % (x, y))
            self.addPoints(x, y)

if __name__ == "__main__":
    import sys
    print("GUI started")
    ph = PlotHandler()
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateFirstPlot)
    timer1.start(30)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()