# stella n2 카메라이미지를 받아서 처리하는것

#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os,rospy,sys,cv2,json,threading
import numpy as np

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from sensor_msgs.msg import Image


class Window11(QWidget):

    def __init__(self,parent):
        super().__init__()        
        # super(Window1, self).__init__()
        # rospy.init_node("Image_node")
        self.cam = None
        rospy.Subscriber("camera",Image,self.callback)


        self.ThreadVideo = VideoWorker(self) #재생
        # self.ThreadVideo.start()
        self.ThreadVideo.changePixmap.connect(self.setImage)
        self.initUI()

    def initUI(self):
  
        self.grblay = QHBoxLayout()
        self.setLayout(self.grblay)
        self.lbl_video = QLabel()

        self.lbl_video.setStyleSheet("background-Color : white")
        self.grblay.addWidget(self.lbl_video)
        # self.resize(self.parent.window1.width(),self.parent.window1.height())
        self.resize(1280,720)
        # self.show()

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        imageArray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.frame = cv2.cvtColor(imageArray, cv2.COLOR_BGR2RGB)
        # frame = cv2.resize(frame,dsize=(msg.width,msg.height),interpolation=cv2.INTER_AREA)
        self.cam = self.frame

        if self.cam is not None:
            self.ThreadVideo.start()
            

        # print(self.cam)
        # print(msg)


    @pyqtSlot(QImage)
    def setImage(self, image):
        
        self.__pixmap = QPixmap(image)
        self.lbl_video.setPixmap(self.__pixmap)
        self.lbl_video.setAlignment(Qt.AlignCenter)


#동영상 쓰레드
class VideoWorker(QThread):#QThread는 gui가 꺼지면, 같이 쓰레드가 종료됩니다. 
    
    changePixmap = pyqtSignal(QImage)
    
    def __init__(self, parent):
        super().__init__(parent)
        # super(VideoWorker, self).__init__(parent)
        self.videoThread = False
        self.parent = parent
       

    def run(self):

            # print(self.parent.cam)
            tmpImage = self.parent.cam
            tmpImage = QImage(tmpImage, tmpImage.shape[1], tmpImage.shape[0], QImage.Format_RGB888)
            self.changePixmap.emit(tmpImage)

        

    def stop(self):
        
        self.videoThread = False
        self.quit()
        self.wait(1000)
    

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = Window11()
    sys.exit(app.exec_())
