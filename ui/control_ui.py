import sys
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton,  QVBoxLayout, QHBoxLayout,QWidget, QTabWidget
import rospy
from PyQt5.QtCore import QTimer

from drive_msgs.msg import *
from std_msgs.msg import Float32MultiArray


from functools import partial

app = None

from libs.widgets import *

STEER_RATIO = 14.6
MPS_TO_KPH = 3.6

def signal_handler(sig, frame):
    QApplication.quit()


class MyApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.sig = 0
        self.merged = 0
        self.sig_in = False

        self.inform = {'e_v':0, 't_v':0, 'e_y':0,'t_y':0, 'e_a':0, 't_a':0, 'e_b':0, 't_b':0}
        self.user_input = Float32MultiArray()
        self.user_input.data = [0,0]



        self.rviz_widget = RvizWidget(self)
        self.rviz_widget.setFixedHeight(1020)
        self.rviz_widget.setFixedWidth(700)
        self.speedometer_widget = SpeedometerWidget(self)
        self.speed_graph = SpeedSubscriberWidget("speed", self)
        self.steer_graph = SpeedSubscriberWidget("steer", self)
        self.wheel_widget = WheelWidget(self)
        self.accel_widget = GaugeWidget("Accel",self)
        self.brake_widget = GaugeWidget("Brake",self)

        self.initUI()

        rospy.Subscriber('/EgoActuator', Actuator, self.ego_actuator_cb)
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)

        self.pub_user_input = rospy.Publisher('/ui/user_input',Float32MultiArray, queue_size=1)
        
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.publish_user_input)
        self.user_input_timer.start(500)

    def ego_actuator_cb(self, msg):
        self.inform['e_y'] = msg.steer.data
        self.inform['e_a'] = msg.accel.data
        self.inform['e_b'] = msg.brake.data
    
    def vehicle_state_cb(self, msg):
        self.inform['e_v'] = int(msg.velocity.data*MPS_TO_KPH)

    def target_actuator_cb(self, msg):
        self.inform['t_y'] = msg.steer.data
        self.inform['t_a'] = msg.accel.data
        self.inform['t_b'] = msg.brake.data
    
    def navigation_data_cb(self, msg):
        if len(msg.plannedVelocity) > 0:
            self.inform['t_v'] = int(msg.plannedVelocity[-1])
        else:
            self.inform['t_v'] = 0

    def set_mode(self, mode):
        self.user_input.data[0] = mode

    def updateUI(self):
        self.speedometer_widget.set_speed(self.inform['e_v'], self.inform['t_v'])
        self.speed_graph.set_speed(self.inform['e_v'], self.inform['t_v'])
        self.steer_graph.set_speed(self.inform['e_y'], self.inform['t_y'])
        self.wheel_widget.set_yaw(self.inform['e_y'], self.inform['t_y'])
        self.accel_widget.set_value(self.inform['e_a'])
        self.brake_widget.set_value(self.inform['e_b'])
        self.accel_widget.set_target(self.inform['t_a'])
        self.brake_widget.set_target(self.inform['t_b'])

    def click_signal(self, v):
        self.user_input.data[1] = v
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(500)
            QTimer.singleShot(5000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.timer.stop()
        self.publish_user_input()

    def publish_user_input(self):
        if self.sig_in:
            self.pub_user_input.publish(self.user_input)
        else:
            self.user_input.data[1] = 0
            self.pub_user_input.publish(self.user_input)

    def initUI(self):

        self.setGeometry(3000,0, 1920, 1080)
        self.setWindowTitle('mobinha-racing: Control')

        tab_widget0 = QTabWidget(self)
        tab0 = QWidget()
        top_buttons_layout = QHBoxLayout()
        b_labels = ['Off', 'On', 'EPS Only', 'ACC Only']
        for i in range(4):
            button = QPushButton(b_labels[i], self)
            button.setFixedHeight(60)
            button.setStyleSheet('font-size: 20px')
            top_buttons_layout.addWidget(button)
            button.clicked.connect(partial(self.set_mode, int(i)))
        button_layout = QVBoxLayout()
        button_layout.addLayout(top_buttons_layout)
        top_buttons_layout2 = QHBoxLayout()
        b2_labels = ['','<', '>', '< >', '^']
        for i in range(1,5):
            button = QPushButton(b2_labels[i], self)
            button.setFixedHeight(60)
            button.setStyleSheet('font-size: 20px')
            top_buttons_layout2.addWidget(button)
            button.clicked.connect(partial(self.click_signal, int(i)))
        button_layout.addLayout(top_buttons_layout2)
        tab0.setLayout(button_layout)

        tab_widget1 = QTabWidget(self)
        tab1 = QWidget()
        cluster_layout = QHBoxLayout()
        cluster_layout.addWidget(self.speedometer_widget)
        cluster_layout.addWidget(self.wheel_widget)
        cluster_layout.addWidget(self.accel_widget)
        cluster_layout.addWidget(self.brake_widget)
        tab1.setLayout(cluster_layout)
        
        tab_widget2 = QTabWidget(self)
        tab2 = QWidget()
        graph_layout = QHBoxLayout()

        speed_layout = QVBoxLayout()
        speed_label = QLabel("Speed Graph")
        speed_label.setFixedHeight(20)
        speed_label.setStyleSheet('font-size: 16px')
        speed_label.setAlignment(Qt.AlignCenter)
        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.speed_graph)

        steer_layout = QVBoxLayout()
        steer_label = QLabel("Steering Graph")
        steer_label.setFixedHeight(20)
        steer_label.setStyleSheet('font-size: 16px')
        steer_label.setAlignment(Qt.AlignCenter)
        steer_layout.addWidget(steer_label)
        steer_layout.addWidget(self.steer_graph)

        # Add the graph layouts to the main graph_layout
        graph_layout.addLayout(speed_layout)
        graph_layout.addLayout(steer_layout)
        tab2.setLayout(graph_layout)

        tab_widget0.addTab(tab0, "Control")
        tab_widget1.addTab(tab1, "Cluster")
        tab_widget2.addTab(tab2, "Graphs")

        central_widget = QWidget(self)
        central_layout = QHBoxLayout()
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.rviz_widget)
        left_layout = QVBoxLayout()
        left_layout.addWidget(tab_widget0)
        left_layout.addWidget(tab_widget1)
        left_layout.addWidget(tab_widget2)
        central_layout.addLayout(left_layout)
        central_layout.addLayout(right_layout)
        central_widget.setLayout(central_layout)
        self.setCentralWidget(central_widget)

def main():
    rospy.init_node('control_ui', anonymous=True)

    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
