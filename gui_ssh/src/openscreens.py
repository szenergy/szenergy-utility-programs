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

from sh import ssh


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
        col, row = (0, 2)
        countRows = len(self.buttonData)//3
        for button in self.buttonData:
            buttonName, buttonLabel, buttonFunction = button["id"], button["label"], button["command"]
            buttonBgColor, buttonTextColor = "rgb(40, 44, 52)", "rgb(171, 178, 191)"
            if "bgColor" in button.keys():
                buttonBgColor = button["bgColor"]
            if "textColor" in button.keys():
                buttonTextColor = button["textColor"]
            
            self.screenButtons[buttonName] = qtgqt.QtGui.QPushButton(buttonLabel)
            if row-2 == countRows:
                if(len(self.buttonData)%3) == 1:
                    widget.addWidget(self.screenButtons[buttonName], row=row, col=1)
                if(len(self.buttonData)%3) == 2:
                    if col==1:
                        col=2
                    widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
            else:
                widget.addWidget(self.screenButtons[buttonName], row=row, col=col)
            buttonFunction = buttonFunction.replace("'","")
            buttonFunction = re.split(r'[,]\s*', buttonFunction)
            print(" ".join(buttonFunction))
            self.screenButtons[buttonName].clicked.connect(partial(self.buttonClicked, buttonFunction))
            self.screenButtons[buttonName].setStyleSheet("background-color: " + buttonBgColor + "; color: " + buttonTextColor)
            if col<2:
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
        self.textArea = qtgqt.QtGui.QTextEdit("127.0.0.1")
        self.textArea.setStyleSheet("color: rgb" + green)
        widg1.addWidget(self.wipeBtn, row=1, col=0)
        widg1.addWidget(self.updateBtn, row=1, col=2)
        widg1.addWidget(self.textArea, row=1, col=1)
        widg1.addWidget(self.sshLabel, row=0, col=1)
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

    def validateIPAddress(self):
        ipList = self.textArea.toPlainText().split('.')
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
            ipAddress = "127.0.0.1"
        
        hostAddress = self.userData['username']+'@'+ipAddress
        sshCommand = []
        # ssh nvidia@192.168.1.5 screen -mdS mc2 bash -c "source ~/.bashrc&& mc"
        if self.allowSSH.isChecked() == True:
            sshCommand.append('ssh')
            sshCommand.append(hostAddress)
            for i in range(0, len(command)-1):
                sshCommand.append(command[i])
            sshCommand.append('`')
            sshCommand.append('bash')
            sshCommand.append('-c')
            sshCommand.append('"')
            sshCommand.append('source ~/.bashrc '+ command[len(command)-1])
            sshCommand.append('"')
            sshCommand.append('`')
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
        
        p = subprocess.Popen(['screen', '-ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)        
        output, err = p.communicate()
        lines = output.splitlines()
        print("Lines:", lines)
        if len(lines) > 2:
            for line in lines:
                line = line.decode('utf-8')
                if line[0] == '\t':
                    self.listwidget.insertItem(0, line.split()[0].strip().split('.')[1])
        
        # SSH Update
        validIP, ipAddress = self.validateIPAddress()
        ipAddress = '.'.join(ipAddress)
        hostAddress = self.userData['username']+'@'+ipAddress

        p = subprocess.Popen(['ssh', hostAddress, 'screen', '-ls'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, err = p.communicate()
        lines = output.splitlines()
        
        if len(lines) > 2:
            for line in lines:
                line = line.decode('utf-8')
                if line[0] == '\t':
                    self.listwidgetSSH.insertItem(0, line.split()[0].strip().split('.')[1])
                

    def openscreen(self):
        item = self.listwidget.currentItem()
        #print(item.text() + " >> double click")        
        toexec = ''.join(['screen -r ', str(item.text()), '; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', toexec])
    
    def openscreenSSH(self):
        item = self.listwidgetSSH.currentItem()
        #print(item.text() + " >> double click")        
        toexec = ''.join(['screen -r ', str(item.text()), '; exec bash'])
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', toexec])

    def listclick(self, qmodelindex):
        item = self.listwidget.currentItem()

    def wipeAllScreens(self):
        self.runningScreens = []
        cmd = ['pkill', 'screen']
        p = subprocess.Popen(cmd)

        validIP, ipAddress = self.validateIPAddress()
        ipAddress = '.'.join(ipAddress)
        hostAddress = self.userData['username']+'@'+ipAddress
        print(ipAddress)

        cmd = ['ssh', hostAddress, 'pkill', 'screen']
        p = subprocess.Popen(cmd)
        print(cmd)
        self.update()