#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os,rospy,sys,cv2,json
import window1,window2,window3,window4,window11

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class integration(QWidget):
    def __init__(self):
        super(integration,self).__init__()
        rospy.init_node('stella_intergration_node')
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle("ROS Manager")
        self.setFixedSize(1600,900)
        
        self.MainLayout=QGridLayout()
        self.setLayout(self.MainLayout)
        
        self.window1=QGroupBox("screen1")
        grblay = QHBoxLayout()
        self.window1.setLayout(grblay)
        self.window1.setFixedSize(780,440)

        w1=window11.Window11(self)
        w2=window2.Window2(self)
        w3=window3.Window3()
        w4=window4.Window4()

        self.window2=QGroupBox("screen2")
        self.window3=QGroupBox("screen3")
        self.window4=QGroupBox("screen4")

        #(1,2,3,4)=(행,열,전체행 길이중에 열의 비중, 전체열 길이중에 열의 비중)
        self.MainLayout.addWidget(self.window1,0,0,1,1)
        self.MainLayout.addWidget(self.window2,0,1,1,1)
        self.MainLayout.addWidget(self.window3,1,0,1,1)
        self.MainLayout.addWidget(self.window4,1,1,1,1)        

        grblay.addWidget(w1)


        grblay = QHBoxLayout()
        self.window2.setLayout(grblay)
        self.window2.setFixedSize(780,440)
        grblay.addWidget(w2)

        grblay = QHBoxLayout()
        self.window3.setLayout(grblay)
        self.window3.setFixedSize(780,440)
        grblay.addWidget(w3)

        grblay = QHBoxLayout()
        self.window4.setLayout(grblay)
        self.window4.setFixedSize(780,440)
        grblay.addWidget(w4)
 
        self.center()
        self.show()        


    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())    

if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = integration()
   sys.exit(app.exec_())