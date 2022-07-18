from csvreader import *
import sys
from jsonreader import JSONReader
from openscreens import *

if __name__ == '__main__':
    reader = JSONReader('ReadFiles/buttons.json')
    print(__file__, "- started ")
    ph = PlotHandler(reader.data["buttons"])
    ph.initializePlot()
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()