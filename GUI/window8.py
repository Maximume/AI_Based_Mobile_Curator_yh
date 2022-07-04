# FileSystemModel을 활용한 파일목록띄우기
# https://www.youtube.com/watch?v=AmvNFDjCW2U&list=PL1eLKSeW1Baj72go6l3gg4C8TXRNUBdMo&index=31
# https://appia.tistory.com/353
# https://stackoverflow.com/questions/49617149/setrootpath-doesnt-set-work-as-expected
import sys, os,shutil

from PIL import Image
from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QWidget,QApplication,QTreeView,QFileSystemModel,QVBoxLayout,QPushButton,QInputDialog,QLineEdit
from PyQt5.QtCore import QFileInfo

class main(QWidget):
    def __init__(self):
        super().__init__()
        self.path = "$/Home"
        # self.path = "$/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/map"
        self.index= None
        

        self.layout = QVBoxLayout()
        self.initUI()
        
    def initUI(self):
        self.model = QFileSystemModel()
        self.tv = QTreeView(self)
        self.btnRen = QPushButton("이름바꾸기")
        self.btnDel = QPushButton("파일삭제")


        self.tv.setModel(self.model)
        self.model.setRootPath(self.path)
        self.tv.setRootIndex(self.model.index("/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/map"))
        
        self.setGeometry(300,300,700,350)
        self.setWindowTitle("지도파일")
        self.tv.setColumnWidth(0,250)

        self.layout.addWidget(self.tv)
        self.layout.addWidget(self.btnDel)
        self.layout.addWidget(self.btnRen)
        self.setLayout(self.layout)




        self.tv.clicked.connect(self.setIndex)
        self.btnRen.clicked.connect(self.ren)
        self.btnDel.clicked.connect(self.rm)
        self.tv.doubleClicked.connect(self.show_image)

        self.center()
        self.show()
        
    
    def show_image(self,index):
        os.chdir(self.model.filePath(self.model.parent(self.index)))
        fname = self.model.fileName(self.index)
        try:
            image=Image.open(fname)
            image.show()
        except:
            print("에러발생") 
        # self.index=index.row()
        # print(self.index)
    # QModelIndex
    def setIndex(self, index):
        self.index=index

        fileInfo:QFileInfo = self.model.fileInfo(index)
        
        f = self.model.fileInfo(index)
        print('filename: ',fileInfo.fileName()) 

    def rm(self):
        os.chdir(self.model.filePath(self.model.parent(self.index)))
        fname = self.model.fileName(self.index) 
        try:
            if not self.model.isDir(self.index):
                os.unlink(fname)
                print(fname+"파일 삭제")
            else:
                shutil.rmtree(fname)
                print(fname+"폴더 삭제")
        except:
            print("에러발생")  

    def ren(self):
        os.chdir(self.model.filePath(self.model.parent(self.index)))
        fname=self.model.fileName(self.index)
        text,res = QInputDialog.getText(self,"이름 바꾸기", "바꿀 이름을 입력하세요.",QLineEdit.Normal,fname)

        if res:
            while True:
                self.ok = True
                for i in os.listdir(os.getcwd()):
                    print(i)
                    if i==text:
                        text,res = QInputDialog.getText(self,"중복 오류", "바꿀 이름을 입력하세요.",QLineEdit.Normal,fname)

                        if not res:
                            return
                            self.ok =False
                if self.ok:
                    break
            os.rename(fname,text)

    

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())


if __name__ == "__main__":
    app=QApplication([])
    ex=main()
    # ex.show()
    sys.exit(app.exec_())
