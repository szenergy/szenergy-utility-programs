#!/usr/bin/env python

from csvreader import *
import sys
from jsonreader import JSONReader
from openscreens import *
import rospkg

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    buttonReader = JSONReader(rospack.get_path("gui_ssh") + '/ReadFiles/buttons.json')
    userDataReader = JSONReader(rospack.get_path("gui_ssh") + '/ReadFiles/userdata.json')
    print(__file__, "- started ")
    ph = PlotHandler(buttonReader.data["buttons"], userDataReader.data["userdata"])
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()