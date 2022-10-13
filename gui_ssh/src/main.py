#!/usr/bin/env python

from csvreader import *
import sys
from jsonreader import JSONReader
from openscreens import *
import os
import rospy

if __name__ == '__main__':
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    username = rospy.get_param('username')
    buttonJSON = rospy.get_param('buttons')
    buttonReader = JSONReader(scriptDir + '/../ReadFiles/' + buttonJSON)
    print(__file__, "- started ")
    ph = PlotHandler(buttonReader.data["buttons"], username)
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()