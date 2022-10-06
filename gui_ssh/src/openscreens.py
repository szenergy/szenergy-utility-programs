#!/usr/bin/env python

# this script uses screen and the default gnome-terminal if not installed, please install with:
# sudo apt install screen gnome-terminal
# sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
# pip install pyqtgraph

from __future__ import print_function
import subprocess
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea
import re
from functools import partial

class PlotHandler(object):
    def __init__(self, buttonData, userData):
        super(PlotHandler, self).__init__()
        self.buttonData = buttonData
        self.userData = userData
        self.screenButtons = {}
        self.runningScreens = []
        pg.setConfigOptions(antialias=True)
        self.app = qtgqt.QtGui.QApplication([])

    def initButtons(self, widget):
        col, row = (0, 4)
        countRows = len(self.buttonData)//5
        for button in self.buttonData:
            buttonName, buttonLabel, buttonFunction = button["id"], button["label"], button["command"]
            buttonBgColor, buttonTextColor = "rgb(40, 44, 52)", "rgb(171, 178, 191)"
            if "bgColor" in button.keys():
                buttonBgColor = button["bgColor"]
            if "textColor" in button.keys():
                buttonTextColor = button["textColor"]
            
            
            self.screenButtons[buttonName] = qtgqt.QtGui.QPushButton(buttonLabel)
            if countRows == row-countRows:
                if(len(self.buttonData)%5 == 2):
                    if col==1:
                        col=4
                    widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
                if(len(self.buttonData)%5 == 3):
                    if col==1:
                        col=2
                    elif col==3:
                        col=4
                    widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
            else:
                widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
            buttonFunction = buttonFunction.replace("'","")
            buttonFunction = re.split(r'[,]\s*', buttonFunction)
            print(" ".join(buttonFunction))
            self.screenButtons[buttonName].clicked.connect(partial(self.buttonClicked, buttonFunction))
            self.screenButtons[buttonName].setStyleSheet("background-color: " + buttonBgColor + "; color: " + buttonTextColor)
            if col<4:
                col+=1
            else:
                col=0
                row+=1
        print(" ")

    def initializePlot(self):
        self.win = qtgqt.QtGui.QMainWindow()
        area = darea.DockArea()
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = "(16, 200, 166)"; greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
        self.win.setWindowTitle("Screen handler")
        self.win.setFixedSize(800, 600)
        self.win.move(600, 200)
        self.win.setCentralWidget(area)
        self.allowSSH = qtgqt.QtGui.QCheckBox("SSH")
        
        dock1 = darea.Dock("", size = (1,1))  # give this dock minimum possible size
        area.addDock(dock1, "left")
        widg1 = pg.LayoutWidget()
        self.initButtons(widg1)
        self.updateBtn = qtgqt.QtGui.QPushButton("update screen list")
        self.wipeBtn = qtgqt.QtGui.QPushButton("wipe screens")
        self.sshLabel = qtgqt.QtGui.QLabel("SSH IP")
        self.sshLabel.setAlignment(qtgqt.QtCore.Qt.AlignCenter)
        self.sshLabel.setMaximumHeight(15)
        self.textArea = qtgqt.QtGui.QTextEdit("192.168.1.5")
        self.textArea.setStyleSheet("color: rgb" + green)
        widg1.addWidget(self.wipeBtn, row=1, col=0)
        widg1.addWidget(self.updateBtn, row=1, col=4)
        widg1.addWidget(self.textArea, row=1, col=2)
        widg1.addWidget(self.sshLabel, row=0, col=2)
        widg1.addWidget(self.allowSSH, row=0, col=0)
        self.textArea.setMaximumHeight(25)
        self.textArea.setMaximumWidth(200)
        widg1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        dock1.setStyleSheet("background-color: rgb(18, 20, 23);")
        dock1.addWidget(widg1)
        self.state = None
        self.updateBtn.clicked.connect(self.update)
        self.wipeBtn.clicked.connect(self.wipeAllScreens)
        self.listwidget = qtgqt.QtGui.QListWidget()
        self.listwidget.setStyleSheet("""QListWidget{ color: rgb(171, 178, 191);}""")
        self.listwidget.clicked.connect(self.listclick)
        self.listwidget.itemDoubleClicked.connect(self.openscreen)
        self.listwidget
        
        self.listwidgetSSH = qtgqt.QtGui.QListWidget()
        self.listwidgetSSH.setStyleSheet("""QListWidget{ color: rgb(171, 178, 191);}""")
        self.listwidgetSSH.clicked.connect(self.listclick)
        self.listwidgetSSH.itemDoubleClicked.connect(self.openscreenSSH)
        self.listwidgetSSH
        
        dock1.addWidget(self.listwidget)
        
        SSHCommandLabel = qtgqt.QtGui.QLabel("SSH Commands")
        SSHCommandLabel.setStyleSheet("""background-color: rgb(24, 28, 31); color:white""")
        dock1.addWidget(SSHCommandLabel)
        dock1.addWidget(self.listwidgetSSH)

        self.update()
        self.timer = qtgqt.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000*10)
        self.win.show()

    def checkIfScreenIsRunning(self):
        p = subprocess.Popen(['screen', '-ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)        
        output, err = p.communicate()
        lines = output.splitlines()
        
        # SSH Update
        validIP, ipAddress = self.validateIPAddress()
        ipAddress = '.'.join(ipAddress)
        hostAddress = self.userData['username']+'@'+ipAddress

        if self.allowSSH.isChecked():
            pSSH = subprocess.Popen(['ssh', hostAddress, 'screen', '-ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)     
            outputSSH, errSSH = pSSH.communicate()
        else:
            outputSSH = ""
        
        linesSSH = outputSSH.splitlines()

        return {'localrun': [lines[0] == 'There is a screen on:' or lines[0] == 'There are screens on:', output],
        'sshrun': [linesSSH[0] == 'There is a screen on:' or linesSSH[0] == 'There are screens on:', outputSSH, hostAddress]}

    def validateIPAddress(self):
        ipList = str(self.textArea.toPlainText())
        ipList = ipList.split('.')
        valid = True

        if len(ipList)!=4 or '' in ipList:
            valid = False
        else:
            for i in ipList:
                if int(i)>255 or int(i)<0 or (len(i)>1 and i[0]=='0'):
                    valid = False
                    break
        
        return valid, ipList

    def buttonClicked(self, command):
        if self.allowSSH.isChecked() == True:
            validIP, ipAddress = self.validateIPAddress()
            
            if(validIP):
                ipAddress = '.'.join(ipAddress)
            else:
                print("Invalid IP address", '.'.join(unicode(ipAddress)))
                return
        else:
            ipAddress = "192.168.1.5"
        
        hostAddress = self.userData['username']+'@'+ipAddress
        sshCommand = []
        # ssh nvidia@192.168.1.5 screen -mdS mc2 bash -c "source ~/.bashrc&& mc"
        if self.allowSSH.isChecked() == True:
            sshCommand.append('ssh')
            sshCommand.append(hostAddress)
            for i in range(0, len(command)-1):
                sshCommand.append(command[i])
            # sshCommand.append('`')
            sshCommand.append('bash')
            sshCommand.append('-c')
            sshCommand.append('"')
            sshCommand.append('source ~/ssh_ros && '+ command[len(command)-1])
            sshCommand.append('"')
            # sshCommand.append('`')
            # sshCommand.append(command[len(command)-1])
            # sshCommand.append('-X')
        else:
            for i in range(0, len(command)-1):
                sshCommand.append(command[i])
            sshCommand.append('bash')
            sshCommand.append('-c')
            sshCommand.append(command[len(command)-1])
        print(" ".join(sshCommand))
        #print(sshCommand)
        #subprocess.check_call(sshCommand) 
        # TODO
        p = subprocess.Popen(sshCommand, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        #print(p.communicate()[0])
        self.update()


    def update(self):
        self.listwidget.clear()
        self.listwidgetSSH.clear()
        
        AllScreens = self.checkIfScreenIsRunning()

        if AllScreens['localrun'][0]:
            lines = AllScreens['localrun'][1].splitlines()
            print("Lines:", lines)
            for i in range(1, len(lines)-1):
                line = lines[i].decode('utf-8') 
                if line[0] == '\t':
                    self.listwidget.insertItem(0, line.split()[0].strip().split('.')[1])
        
        # SSH Update
        if self.allowSSH.isChecked():
            validIP, ipAddress = self.validateIPAddress()
            ipAddress = '.'.join(ipAddress)
            hostAddress = self.userData['username']+'@'+ipAddress
            
            if AllScreens['sshrun'][0]:
                lines = AllScreens['sshrun'][1].splitlines()
                print("Lines:", lines)
                for i in range(1, len(lines)-1):
                    line = lines[i].decode('utf-8')
                    if line[0] == '\t':
                        self.listwidgetSSH.insertItem(0, line.split()[0].strip().split('.')[1])
                

    def openscreen(self):
        item = self.listwidget.currentItem()
        #print(item.text() + " >> double click")        
        toexec = ''.join(['screen -r ', str(item.text()), '; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', toexec])
    
    def openscreenSSH(self):
        item = self.listwidgetSSH.currentItem()

        validIP, ipAddress = self.validateIPAddress()
        ipAddress = '.'.join(ipAddress)
        hostAddress = self.userData['username']+'@'+ipAddress

        #print(item.text() + " >> double click")        
        toexec = ''.join(['ssh -t ', hostAddress, ' screen -r ', str(item.text()), '; exec bash'])
        print(toexec)
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', toexec])

    def listclick(self, qmodelindex):
        item = self.listwidget.currentItem()

    def wipeAllScreens(self):
        self.runningScreens = []

        AllScreens = self.checkIfScreenIsRunning()

        if AllScreens['localrun'][0]:
            lines = AllScreens['localrun'][1].splitlines()
            # print("Lines:", lines)
            for i in range(1, len(lines)-1):
                line = lines[i].decode('utf-8')
                PID = line.split()[0].strip().split('.')[1]
                p = subprocess.Popen(['screen', '-XS', PID, 'quit'])
        
        if self.allowSSH.isChecked():
            if AllScreens['sshrun'][0]:
                lines = AllScreens['sshrun'][1].splitlines()
                print("Lines:", lines)
                for i in range(1, len(lines)-1):
                    line = lines[i].decode('utf-8')
                    print("Actual Line:", lines[i])
                    PID = line.split()[0].strip().split('.')[1]
                    p = subprocess.Popen(['ssh', '-t', AllScreens['sshrun'][2], 'screen', '-XS', PID, 'quit'])
        
        self.update()