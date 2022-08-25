from csvreader import *
import sys
from jsonreader import JSONReader
from openscreens import *

if __name__ == '__main__':
    buttonReader = JSONReader('ReadFiles/buttons.json')
    userDataReader = JSONReader('ReadFiles/userdata.json')
    print(__file__, "- started ")
    ph = PlotHandler(buttonReader.data["buttons"], userDataReader.data["userdata"])
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()