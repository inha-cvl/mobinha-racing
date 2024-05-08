import sys
import random
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from functools import partial

import rospy
from std_msgs.msg import Float32, Int8
from drive_msgs.msg import *

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)
        self.setParent(parent)

    def plot(self, current_velocity, target_velocity):
        self.axes.clear()
        self.axes.plot(current_velocity, label='Current Velocity')
        self.axes.plot(target_velocity, label='Target Velocity')
        self.axes.legend(loc='upper right')
        self.axes.set_title('Velocity Over Time')
        self.draw()

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.current_vel = [0]
        self.control_target_vel = 0.0
        self.target_vel = [0]

        self.ego_actuator = [0,0,0]
        self.target_actuator = [0,0,0]
        
        self.time = 0

        rospy.init_node("simple_ui")
        
        self.initUI()

        rospy.Subscriber('/EgoActuator', Actuator, self.ego_actuator_cb)
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/control/target_velocity', Float32, self.target_velocity_cb)
        self.pub_user_target_velocity = rospy.Publisher('/test/target_velocity', Float32, queue_size=1)
        self.pub_system_mode = rospy.Publisher('/state_machine/system_state',Int8, queue_size=1)

        self.initTimer()
    

    def ego_actuator_cb(self, msg):
        self.current_steer_label.setText(f'Current Steer: {round(msg.steer.data, 2)}')
        self.current_accel_label.setText(f'Current Accel: {round(msg.accel.data,2)}') 
        self.current_brake_label.setText(f'Current Brake: {round(msg.brake.data,2)}')
    
    def target_actuator_cb(self, msg):
        self.current_steer_label.setText(f'Current Steer: {round(msg.steer.data,2)}')
        self.current_accel_label.setText(f'Current Accel: {round(msg.accel.data,2)}') 
        self.current_brake_label.setText(f'Current Brake: {round(msg.brake.data,2)}')
    
    def vehicle_state_cb(self, msg):
        self.current_vel.append(round(msg.velocity.data*3.6,2))
        self.target_vel.append(round(self.control_target_vel*3.6,2))
        self.current_velocity_label.setText(f'Current Velocity: {round(msg.velocity.data,2)}')
    
    def target_velocity_cb(self, msg):
        self.control_target_vel = msg.data
    
    def initUI(self):
        # 전체 수직 레이아웃
        main_layout = QVBoxLayout()

        # 상단의 네 개의 버튼
        top_buttons_layout = QHBoxLayout()
        b_labels = ['Off', 'On', 'EPS Only', 'ACC Only']
        for i in range(4):
            button = QPushButton(b_labels[i], self)
            top_buttons_layout.addWidget(button)
            button.clicked.connect(partial(self.mode_publish, int(i)))
        main_layout.addLayout(top_buttons_layout)

        # 현재 Velocity와 Target Velocity 입력
        velocity_layout = QHBoxLayout()
        self.current_velocity_label = QLabel(f'Current Velocity: {self.current_vel[-1]}', self)
        self.target_velocity_input = QLineEdit(self)
        self.send_velocity_button = QPushButton("Set Target Velocity", self)
        self.send_velocity_button.clicked.connect(self.update_velocity)

        velocity_layout.addWidget(self.current_velocity_label)
        velocity_layout.addWidget(self.target_velocity_input)
        velocity_layout.addWidget(self.send_velocity_button)
        main_layout.addLayout(velocity_layout)

        # 현재 Steer, Accel, Brake 값
        current_values_layout = QHBoxLayout()
        self.current_steer_label = QLabel(f'Current Steer: {self.ego_actuator[0]}', self)
        self.current_accel_label = QLabel(f'Current Accel: {self.ego_actuator[1]}', self)
        self.current_brake_label = QLabel(f'Current Brake: {self.ego_actuator[2]}', self)

        current_values_layout.addWidget(self.current_steer_label)
        current_values_layout.addWidget(self.current_accel_label)
        current_values_layout.addWidget(self.current_brake_label)
        main_layout.addLayout(current_values_layout)

        # 목표 Steer, Accel, Brake 값
        target_values_layout = QHBoxLayout()
        self.target_steer_label = QLabel(f'Target Steer: {self.target_actuator[0]}', self)
        self.target_accel_label = QLabel(f'Target Accel: {self.target_actuator[1]}', self)
        self.target_brake_label = QLabel(f'Target Brake: {self.target_actuator[2]}', self)

        target_values_layout.addWidget(self.target_steer_label)
        target_values_layout.addWidget(self.target_accel_label)
        target_values_layout.addWidget(self.target_brake_label)
        main_layout.addLayout(target_values_layout)

        # 그래프 캔버스 추가
        self.canvas = PlotCanvas(self, width=5, height=4)
        main_layout.addWidget(self.canvas)

        # 메인 윈도우 설정
        self.setLayout(main_layout)
        self.setWindowTitle('Control Panel')
        self.setGeometry(300, 300, 600, 400)

    def mode_publish(self, mode):
        self.pub_system_mode.publish(mode)
   
    def initTimer(self):
        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # Update every second
        self.timer.timeout.connect(self.update_graph)
        self.timer.start()

    def update_velocity(self):
        target_velocity = self.target_velocity_input.text()
        try:
            target_velocity = float(target_velocity)/3.6
            self.pub_user_target_velocity.publish(target_velocity)
        except ValueError:
            pass
    
    def update_graph(self):
        self.canvas.plot(self.current_vel, self.target_vel)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MainWindow()
    ex.show()
    sys.exit(app.exec_())
