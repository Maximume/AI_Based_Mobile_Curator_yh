# 속도 및 자율주행 최종 클래스
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
import numpy as np
import os,sys,rospy,cv2,threading,window8,signal
import time,json,time,datetime,math,io

from asyncio.subprocess import PIPE
from signal import SIGINT, signal
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

from subprocess import call
from subprocess import Popen  
import subprocess

from PyQt5.QtWidgets import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
from PyQt5.QtCore import *


from geometry_msgs.msg import Twist

dirpath=os.path.dirname(os.path.abspath(__file__))


imagePath = os.path.join(dirpath,'photo')
# print(imagePath)
if not os.path.exists(imagePath):
    os.makedirs(imagePath)

file_path = "/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/final.json"

# 기존 json 파일 읽어오기
with open(file_path, 'r') as file:
    data = json.load(file)

    displayList = data['List']

#동영상 캡쳐
fourcc = cv2.VideoWriter_fourcc(*'XVID')
record = False

cnt=0
class Window3(QWidget):
    def __init__(self):
        # super().__init__()
        super(Window3, self).__init__()
        self.initUI()
        self.proc = None
        self.w=None
        rospy.Subscriber("camera",Image,self.callback)
        rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.callback2)
        # rospy.Subscriber("/odom",Odometry,self.callback2)
        rospy.Subscriber("qr_topic",String,self.callback3)
        self.now = datetime.datetime.now().strftime("%d_%H-%M-%S")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist=Twist()

    def initUI(self):
        tab1 = QWidget()
        tab2 = QWidget()
        self.resize(800,600)

        tabs = QTabWidget()
        tabs.addTab(tab1, 'Manual Mode')
        tabs.addTab(tab2, 'Auto Move')

        a=tabs.currentIndex()
        
        vbox = QVBoxLayout()
        self.setLayout(vbox)
        vbox.addWidget(tabs)

            
        self.Follow_WALL= QPushButton("Follow WALL")
        self.Save_Follow_WALL = QPushButton("Save_Follow_WALL")
        self.view_Map = QPushButton("View_save_map")

        self.Follow_WALL.clicked.connect(self.onStart)
        self.Save_Follow_WALL.clicked.connect(self.endProgram)
        self.view_Map.clicked.connect(self.viewProgram)

        vbox1 = QHBoxLayout()
        tab2.setLayout(vbox1)


        vbox1.addWidget(self.Follow_WALL)
        vbox1.addWidget(self.Save_Follow_WALL)
        vbox1.addWidget(self.view_Map)


        tab1_vbox=QVBoxLayout()
        tab1_hbox1=QHBoxLayout()
        tab1_hbox2=QHBoxLayout()
        
        self.btn_straight = QPushButton("▲")
        tab1_vbox.addWidget(self.btn_straight)
        tab1.setLayout(tab1_vbox)

        self.btn_left = QPushButton("◀")
        tab1_hbox1.addWidget(self.btn_left)
        self.btn_back = QPushButton("▼")  
        tab1_hbox1.addWidget(self.btn_back)
        self.btn_right = QPushButton("▶")
        tab1_hbox1.addWidget(self.btn_right)
        tab1_vbox.addLayout(tab1_hbox1)

        self.btn_break = QPushButton("Capture")
        tab1_hbox2.addWidget(self.btn_break)
        tab1_vbox.addLayout(tab1_hbox2)

        
        self.btnMoveGroup = QButtonGroup()
        self.btnMoveGroup.addButton(self.btn_straight,1)
        self.btnMoveGroup.addButton(self.btn_left,2)
        self.btnMoveGroup.addButton(self.btn_right,3)
        self.btnMoveGroup.addButton(self.btn_back,4)
        self.btnMoveGroup.addButton(self.btn_break,5)

        self.btnMoveGroup.setExclusive(True)
        self.btnMoveGroup.buttonPressed[int].connect(self.keyPressEvent)
        self.btnMoveGroup.buttonReleased[int].connect(self.keyReleaseEvent)
        # self.show()

    def keyPressEvent(self, e):
        if e.key() ==Qt.Key_W:
            self.twist.linear.x=0.3
            self.pub.publish(self.twist)
            self.btn_straight.setStyleSheet("background-Color : yellow")

        if e.key() ==Qt.Key_S:
            self.twist.linear.x=-0.3
            self.pub.publish(self.twist)
            self.btn_back.setStyleSheet("background-Color : yellow")


        if e.key() ==Qt.Key_D:
            self.twist.angular.z=-0.8
            self.pub.publish(self.twist)
            self.btn_right.setStyleSheet("background-Color : yellow")


        if e.key() ==Qt.Key_A:
            self.twist.angular.z=0.8
            self.pub.publish(self.twist)

            self.btn_left.setStyleSheet("background-Color : yellow")


        if e.key() ==Qt.Key_T:
            global cnt
            self.twist.linear.x=0.0
            self.twist.angular.z=0.0
            self.pub.publish(self.twist)
            self.btn_break.setStyleSheet("background-Color : red")
            now = datetime.datetime.now().strftime("%d_%H-%M-%S")
            cv2.imwrite('/home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/photo/'+str(now)+".png", self.imageArray)
            
            if self.bool is True:
                for self.i in range(len(data["List"])):
                    if self.qr_code in data["List"][self.i]["name"]:
                        print(self.qr_code)
                        data["List"][self.i]["Num"]=cnt
                        data["List"][self.i]["image"]=str(now)+".png"
                        data["List"][self.i]["create"]=str(now)
                        data["List"][self.i]["Pose_pos"]=self.pos
                        data["List"][self.i]["Pose_qut"]=self.euler_angle
                        print(data["List"][self.i])
                        cnt+=1
                        with io.open(file_path, 'w', encoding='utf-8') as file:
                            json.dump(data, file, indent="\t",ensure_ascii=False)
            self.bool=False
                
    def callback(self, msg):
        # rospy.loginfo('Image received...')
        # 이 친구는 RGB 순서로 들어온다.
        self.imageArray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        
    def callback2(self,data_pos):
        
        self.pos=list()
        self.pos.append(round(data_pos.pose.pose.position.x,4)) #현재위치 (x,y)
        self.pos.append(round(data_pos.pose.pose.position.y,4)) #현재위치 (x,y)
        self.pos.append(round(data_pos.pose.pose.position.z,4)) #현재위치 (x,y)
        # print(pos)

        self.qut=[]
        qut_x=data_pos.pose.pose.orientation.x #현재각도 (z,w)
        qut_y=data_pos.pose.pose.orientation.y #현재각도 (z,w)
        qut_z=round(data_pos.pose.pose.orientation.z,4) #현재각도 (z,w)
        qut_w=round(data_pos.pose.pose.orientation.w,4) #현재각도 (z,w)
        self.qut.append(qut_x)
        self.qut.append(qut_y)
        self.qut.append(qut_z)
        self.qut.append(qut_w)
        
        #transform quan to degree
        self.euler_angle = round(self.euler_from_quaternion_degrees(qut_x,qut_y,qut_z,qut_w),4)

    def callback3(self, msg):
        self.qr_code=msg.data
        self.bool=True

    def euler_from_quaternion_degrees(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        roll_x = roll_x * 180 / math.pi
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        pitch_y = pitch_y * 180 / math.pi
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        yaw_z = yaw_z * 180 / math.pi
        #print(“Roll={}, Pitch={}, Yaw={}“.format(roll_x, pitch_y, yaw_z))
        return yaw_z

    #     #키보드 조작
    def keyReleaseEvent(self, e):

        self.btn_straight.setStyleSheet("background-Color : white")
        self.btn_back.setStyleSheet("background-Color : white")
        self.btn_right.setStyleSheet("background-Color : white")
        self.btn_left.setStyleSheet("background-Color : white")
        
        if e.key() == Qt.Key_W and not e.isAutoRepeat():
            self.twist.linear.x=0.0
            self.pub.publish(self.twist)

            

        if e.key() == Qt.Key_S and not e.isAutoRepeat():
            self.twist.linear.x=0.0
            self.pub.publish(self.twist)

            

        if e.key() == Qt.Key_A and not e.isAutoRepeat():
            self.twist.angular.z=0.0
            self.pub.publish(self.twist)


        if e.key() == Qt.Key_D and not e.isAutoRepeat():
            self.twist.angular.z=0.0
            self.pub.publish(self.twist)

        
        if e.key() == Qt.Key_T and not e.isAutoRepeat():
            self.bool=False

            
    
#마우스로 버튼을 눌렀을때 실행되는 코드, 기능구현 안해놓음
    # def btnClicked(self, id):
    #     for button in self.btnMoveGroup.buttons():
    #         if button is self.btnMoveGroup.button(id):
    #             # self.label.setText(button.text() + " Clicked")
    #             print(button.text() + " Clicked!")

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    # def onStart(self):
    #     ROS_PROGRAM = QProcess(self)
    #     program = 'roslaunch stella_slam wall_follower.launch'
    #     ROS_PROGRAM.start(program)


    def onStart(self):
        # self.proc = subprocess.Popen(["rosrun", "stella_slam", "test1.py"])
    
        self.proc = subprocess.Popen(["roslaunch", "stella_slam", "stella_slam.launch"],stdout=PIPE,stderr=PIPE)
        self.proc = subprocess.Popen(["roslaunch", "stella_slam", "wall_follower.launch"],stdout=PIPE,stderr=PIPE)

    def endProgram(self):
        now = datetime.datetime.now().strftime("%H-%M")
        os.system("cd /home/aa/catkin_ws/src/STELLA_REMOTE_PC_N2/stella_slam/src/map&&rosrun map_server map_saver -f "+ str(now))
        self.proc.terminate()

    def viewProgram(self):
        if self.w is None:
            self.w = window8.main()
            self.w.show()

        else:
            self.w.close()  # Close window.
            self.w = None 
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = Window3()
    sys.exit(app.exec_())

# 부모프로세스->자식프로세스 순으로 실행
# self.proc.kill() or termainate() 부모만 죽어서
# 밑에 있는 자식프로세스는 좀비프로세스가 된다.

# 1.자식프로세스에 대한 종료가 있을때까지 부모를 종료하지 않는다.
# 2.자식프로세스에 대한 pid를 찾아서 프로세스 종료순서를 다시 잡아준다.