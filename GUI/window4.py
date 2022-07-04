#!/usr/bin/python3
# -*- coding: utf-8 -*-


import os,sys,rospy,cv2,json,io,time


from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *


dirpath=os.path.dirname(os.path.abspath(__file__))

jsonPath = os.path.join(dirpath,'food.json')

file_path = "/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/final.json"

### 사이즈 정책을 설정한 새로운 class를 생성합니다. ###
class QPushButton(QPushButton):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

class QLabel(QLabel):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

class QTextEdit(QTextEdit):
    def __init__(self, parent = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

class Window4(QWidget):
    data_list1=[]
    data_list2=[]
    data={}
    index1=None
    index2=None
    def __init__(self):
        super().__init__()
        # super(Window2,self).__init()
        self.initUI()
    
    def initUI(self):
        mainlayout=QVBoxLayout()

        operation_first_layout=QHBoxLayout()
        operation_second_layout=QHBoxLayout()
        
        first_first_layout=QVBoxLayout()
        second_first_layout=QVBoxLayout()
        third_first_layout=QVBoxLayout()
        
        self.exhibition=QFormLayout()
        self.destination=QFormLayout()
        
        self.listview1 = QListView()
        
        self.model1 = QStandardItemModel()
        self.listview1.setModel(self.model1)
        
        self.listview2 = QListView()
        self.model2 = QStandardItemModel()
        self.listview2.setModel(self.model2)
        
        self.add=QPushButton(">>")
        self.load=QPushButton("불러오기")
        self.remove=QPushButton("<<")
        

        mainlayout.addLayout(operation_first_layout,stretch=8)
        mainlayout.addLayout(operation_second_layout,stretch=2)
        
        operation_first_layout.addLayout(first_first_layout)
        operation_first_layout.addLayout(second_first_layout)
        operation_first_layout.addLayout(third_first_layout)

        label1=QLabel("전시품")
        label1.setAlignment(Qt.AlignCenter)
        # label1.setStyleSheet("background-Color : white")
        label2=QLabel("방문지")
        label2.setAlignment(Qt.AlignCenter)

        self.exhibition.addRow(label1)
        self.destination.addRow(label2)

        
        first_first_layout.addStretch(1)
        first_first_layout.addLayout(self.exhibition,stretch=2)
        first_first_layout.addWidget(self.listview1,stretch=10)

        third_first_layout.addStretch(1)
        third_first_layout.addLayout(self.destination,stretch=2)
        third_first_layout.addWidget(self.listview2,stretch=10)

        self.selModel1=self.listview1.selectionModel()
        self.selModel2=self.listview2.selectionModel()
        
        # QItemSelectionModel
        self.selModel1.selectionChanged.connect(self.onClicked1)
        self.selModel2.selectionChanged.connect(self.onClicked2)

        self.listview1.clicked.connect(self.listclicked1)
        self.listview2.clicked.connect(self.listclicked2)



        self.load.clicked.connect(self.load_display)
        self.add.clicked.connect(self.load2destination)
        self.remove.clicked.connect(self.destination2load)

        second_first_layout.addStretch(1)
        second_first_layout.addWidget(self.load,alignment=Qt.AlignCenter)
        second_first_layout.addStretch(3)
        second_first_layout.addWidget(self.add)
        second_first_layout.addStretch(3)
        second_first_layout.addWidget(self.remove)
        second_first_layout.addStretch(3)

        operation_second_layout.addStretch(1)
        self.start=QPushButton("시작")

        operation_second_layout.addWidget(self.start,stretch=1,alignment=Qt.AlignCenter)
        operation_second_layout.addStretch(1)
        self.start.clicked.connect(self.load_destionation)
        
        self.setLayout(mainlayout)
        self.resize(800,600)
        self.show()


    def load_display(self):
        self.model1.clear()
        self.model2.clear()
        self.data_list1.clear()
        self.data_list2.clear()

        

        # 기존 json 파일 읽어오기
        with open(file_path, 'r') as file:
            data = json.load(file)

            displayList = data['List']

            for f in displayList:
                self.model1.appendRow(QStandardItem(f['name']))
                self.data_list1.append(f)


    def load_destionation(self):

        pos_arr=list()
        qut_arr=list()
        for i in range(len(self.data_list2)):
            for j in range(3):
                pos_arr.append(self.data_list2[i]["Pose_pos"][j])
            qut_arr.append(self.data_list2[i]["Pose_qut"])
            
        QMessageBox.about(self, "박물관 전시품", " 이동중.. ")

        self.data_list1.clear()
        self.data_list2.clear()
        self.load_display()


    def load2destination(self):
        self.data = self.data_list1[self.index1]
        self.data_list2.append(self.data)
        self.model2.appendRow(QStandardItem(self.data['name']))
        self.data_list1.pop(self.index1)
        self.model1.takeRow(self.index1)
        self.index1=self.listview1.selectedIndexes()[0].row()


    def destination2load(self):
        self.data = self.data_list2[self.index2]
        self.data_list1.append(self.data)
        self.model1.appendRow(QStandardItem(self.data['name']))
        self.data_list2.pop(self.index2)
        self.model2.takeRow(self.index2)
        self.index2=self.listview2.selectedIndexes()[0].row()


    def listclicked1(self,index:QModelIndex):
        self.index1=index.row()
        print("selected: ",self.index1)
        
    
    def listclicked2(self,index:QModelIndex):
        self.index2=index.row()
        print("selected: ",self.index2)
        

    
    def onClicked1(self,selected,deselected):

        self.index1=selected.indexes()[0].row()

    
    def onClicked2(self,selected,deselected):

        self.index2=selected.indexes()[0].row()


if __name__=='__main__':

   app = QApplication(sys.argv)
   ex = Window4()
   sys.exit(app.exec_())