# 리스트뷰를 이용한 UI 보여주기

#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os,sys,rospy,cv2,json,io


from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *


dirpath=os.path.dirname(os.path.abspath(__file__))

jsonPath = os.path.join(dirpath,'food.json')

imagePath = os.path.join(dirpath,'photo')

#동영상 캡쳐
fourcc = cv2.VideoWriter_fourcc(*'XVID')
record = False

#######################################
if not os.path.exists(imagePath):
    os.makedirs(imagePath)

file_path = "/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/final.json"

# 기존 json 파일 읽어오기
with open(file_path, 'r') as file:
    data = json.load(file)

    displayList = data['List']
########################################

### 사이즈 정책을 설정한 새로운 class를 생성합니다. ###
class QPushButton(QPushButton):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

class QLabel(QLabel):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

class QLineEdit(QLineEdit):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)    

class Window2(QWidget):
    def __init__(self,parent):
        super().__init__(parent)
        # super(Window2,self).__init()
        self.initUI()
    
    def initUI(self):
        mainlayout=QVBoxLayout()

        title_layout=QHBoxLayout()
        operaiton_layout=QHBoxLayout()
        operaiton_layout1=QVBoxLayout()
        operaiton_layout2=QVBoxLayout()
        self.art_explain = QFormLayout()

        #제목표시 하기 위한 라벨
        self.title=QLabel("What is in Musium")
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet("background-Color : white")
        title_layout.addWidget(self.title)

        self.listview = QListView()
        self.model = QStandardItemModel()
        self.listview.setModel(self.model)
        self.listview.clicked.connect(self.listclicked)
        

        label1=QLabel("작품이름")
        label1.setAlignment(Qt.AlignCenter)

        label2=QLabel("생성년도")
        label2.setAlignment(Qt.AlignCenter)

        label3=QLabel("작품설명")
        label3.setAlignment(Qt.AlignCenter)

        self.label4=QLineEdit("")
        self.label4.setAlignment(Qt.AlignCenter)
        self.label4.setReadOnly(True)

        self.label5=QLineEdit("")
        self.label5.setAlignment(Qt.AlignCenter)
        self.label5.setReadOnly(True)

        self.label6=QTextEdit("")
        self.label6.setAlignment(Qt.AlignCenter)
        self.label6.setReadOnly(True)

        self.label7=QLabel("")
        self.label7.setStyleSheet("background-Color : white")

        self.label8=QHBoxLayout()
        

        self.button=QPushButton("새로고침")
        self.label8.addWidget(self.button)
        self.button.clicked.connect(self.reclear)

        self.art_explain.addRow(label1,self.label4)
        self.art_explain.addRow(label2,self.label5)
        self.art_explain.addRow(label3)
        self.art_explain.addRow(self.label6)
        
        operaiton_layout.addLayout(operaiton_layout1,stretch=2)
        operaiton_layout.addLayout(operaiton_layout2,stretch=3)

        self.selModel=self.listview.selectionModel()
        self.selModel.selectionChanged.connect(self.onClicked)
        
        operaiton_layout1.addWidget(self.listview,stretch=3,)
        operaiton_layout1.addLayout(self.art_explain,stretch=5)

        operaiton_layout2.addWidget(self.label7,stretch=8)
        operaiton_layout2.addLayout(self.label8,stretch=2)

        
        mainlayout.addLayout(title_layout,stretch=1)
        mainlayout.addLayout(operaiton_layout,stretch=8)

        self.resize(800,600)
        self.setLayout(mainlayout)
        self.show()


    def reclear(self):
        global displayList
        
        self.model.clear()

        with open(file_path) as file:
            data = json.load(file)

            displayList = data['List']

        for f in displayList:
            self.model.appendRow(QStandardItem(f['name']))
            self.listview.setModel(self.model)
        
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def listclicked(self,index:QModelIndex):
        data=displayList[index.row()]
        # print(index.row())
        self.label4.setText(data['name'])
        self.label5.setText(data['create'])
        self.label6.setText(data["explane"])
  

        imagepath=data["image"]

        resPath = os.path.join(imagePath, imagepath)
        image = QImage(resPath)
        self.label7.setPixmap(QPixmap.fromImage(image).scaled(image.height(),image.width(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.label7.setAlignment(Qt.AlignCenter)
        self.label7.setStyleSheet("background-Color : white")


    def onClicked(self,selected,deselected):

        index=selected.indexes()[0].row()
        data = displayList[index]
        self.label4.setText(data['name'])
        self.label5.setText(data['create'])
        self.label6.setText(data["explane"])
  

        imagepath=data["image"]

        resPath = os.path.join(imagePath, imagepath)

        image = QImage(resPath)
        self.label7.setPixmap(QPixmap.fromImage(image).scaled(image.height(),image.width(),Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.label7.setAlignment(Qt.AlignCenter)
        self.label7.setStyleSheet("background-Color : white")

if __name__=='__main__':

   app = QApplication(sys.argv)
   ex = Window2()
   sys.exit(app.exec_())