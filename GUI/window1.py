# 실시간 카메라 최종본(웹캠용)

#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os,rospy,sys,cv2,json,threading

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class Window1(QWidget):
    def __init__(self,parent):
        # super().__init__()
        super(Window1, self).__init__(parent)
        self.parent=parent
        self.initUI()

    def initUI(self):
        self.ThreadVideo = VideoWorker(self) #재생
        self.ThreadVideo.start()
        self.ThreadVideo.changePixmap.connect(self.setImage)


        self.grblay = QHBoxLayout()
        self.setLayout(self.grblay)
        self.lbl_video = QLabel()
        self.lbl_video.setStyleSheet("background-Color : black")
        self.grblay.addWidget(self.lbl_video)
        
        # self.resize(self.parent.width(),self.parent.height())
        # self.resize(1280,720)
        # self.show()

    @pyqtSlot(QImage)
    def setImage(self, image):
        
        pixmap = QPixmap(image)
       
        self.lbl_video.setPixmap(pixmap)
        self.lbl_video.setAlignment(Qt.AlignCenter)


#동영상 쓰레드
class VideoWorker(QThread):#QThread는 gui가 꺼지면, 같이 쓰레드가 종료됩니다. 
    
    changePixmap = pyqtSignal(QImage)
    
    def __init__(self, parent):
        # super().__init__(parent)
        super(VideoWorker, self).__init__(parent)
        self.videoThread = False
        self.parent=parent
    def run(self):
        
        cap = cv2.VideoCapture(0)#1280 * 720 1.778
        # cap = cv2.VideoCapture(2)#1920 * 1080
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,5000)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,5000)
        self.width=cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
        self.width=int(self.width)
        self.height=int(self.height)
        # 부모의 너비,높이 가져오기
        # width=self.parent.width()
        # height=self.parent.height()

        while cap.isOpened() and self.parent:
            ret, frame = cap.read()
                
            if not ret:
                    
                pass

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # frame=cv2.resize(frame,dsize=(width,int(width/1.777)),interpolation=cv2.INTER_AREA)
            frame=cv2.resize(frame,dsize=(self.width,self.height),interpolation=cv2.INTER_AREA)
            tmpImage = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888)
            p = tmpImage.scaled(self.height, self.width, Qt.KeepAspectRatio,Qt.SmoothTransformation)
            # self.changePixmap.emit(tmpImage)
            self.changePixmap.emit(p)
            
    def stop(self):
        
        self.videoThread = False
        self.quit()
        self.wait(1000)

if __name__ == '__main__':
   app = QApplication(sys.argv)
   ex = Window1()
   sys.exit(app.exec_())
