#!/usr/bin/env python

from csvreader import *
import sys
from jsonreader import JSONReader
from openscreens import *
import os

if __name__ == '__main__':
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    buttonReader = JSONReader(scriptDir + '/../ReadFiles/buttons.json')
    userDataReader = JSONReader(scriptDir + '/../ReadFiles/userdata.json')
    print(__file__, "- started ")
    ph = PlotHandler(buttonReader.data["buttons"], userDataReader.data["userdata"])
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()