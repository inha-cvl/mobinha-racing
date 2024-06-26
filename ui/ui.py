import sys
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from functools import partial

app = None

from libs.widgets import *
from ros_handler import ROSHandler

form_class = uic.loadUiType("./forms/mainwindow.ui")[0]

STEER_RATIO = 12.9
MPS_TO_KPH = 3.6

def signal_handler(sig, frame):
    QApplication.quit()


class MyApp(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.RH = ROSHandler()

        self.set_values()
        self.set_widgets()
        self.set_timers()

    def set_values(self):
        self.sig_in = False
        self.mode_strings = ['Autonomous Driving OFF','Auotnomous Driving ON', 'EPS Only Mode', 'ACC Only Mode']
        self.signal_strings = ['Off', 'Left Change', 'Right Change', 'Hazard', 'Straight']
        
    def set_widgets(self):
        self.rviz_widget = RvizWidget(self)
        self.speedometer_widget = SpeedometerWidget(self)
        self.speed_graph = SpeedSubscriberWidget("speed", self)
        self.steer_graph = SpeedSubscriberWidget("steer", self)
        self.wheel_widget = WheelWidget(self)
        self.gear_widget = GearWidget(self)
        self.initUI()
    
    def set_timers(self):
        self.timer = QTimer(self)   
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(100)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.RH.publish)
        self.user_input_timer.start(500)


    def updateUI(self):
        self.speedometer_widget.set_speed(self.RH.ego_value['velocity'], self.RH.target_value['velocity'])
        self.speed_graph.set_speed(self.RH.ego_value['velocity'], self.RH.target_value['velocity'])
        self.steer_graph.set_speed(self.RH.ego_value['steer'], self.RH.target_value['steer'])
        self.wheel_widget.set_yaw(self.RH.ego_value['steer'], self.RH.target_value['steer'])
        self.gear_widget.set_gear(self.RH.ego_value['gear'])

        self.system_label_update(self.RH.system_status['mode'], self.RH.system_status['signal'],
                                 self.RH.lane_number, self.RH.system_status['lap_count'])

        self.can_table_update(self.RH.can_inform)
        
    def system_label_update(self, mode, signal, lane_number, lap_count):
        self.systemLabel1.setText(self.mode_strings[int(mode)])
        self.systemLabel2.setText(self.signal_strings[int(signal)])
        self.laneNumberLabel.setText(f"Lane: {lane_number}")
        self.lapCountLabel.setText(f"Lap: {lap_count}")
    
    def can_table_update(self, can_inform):
        self.canTable.setItem(0, 1, QTableWidgetItem(can_inform['eps_status']))
        self.canTable.setItem(1, 1, QTableWidgetItem(can_inform['acc_status']))


    def click_mode(self, mode):
        self.RH.user_value['user_mode'] = mode
        self.check_timer()
    
    def click_signal(self, v):
        self.RH.user_value['user_signal'] = v
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(500)
            QTimer.singleShot(3000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RH.user_value['user_signal'] = 0
        self.user_input_timer.stop()
        self.RH.publish()

    def initUI(self):
        self.set_conntection()

        self.clusterSpeedLayout.addWidget(self.speedometer_widget)
        self.clusterSteerLayout.addWidget(self.wheel_widget)
        self.clusterLeftDownLayout.addWidget(self.gear_widget)

        self.speedLayout.addWidget(self.speed_graph)
        self.steerLayout.addWidget(self.steer_graph)
        
        self.rvizLayout.addWidget(self.rviz_widget)

    def set_conntection(self):
        top_buttons1 = [self.buttonOff, self.buttonOn, self.buttonACCOnly, self.buttonEPSOnly]
        for i, button in enumerate(top_buttons1):
            button.clicked.connect(partial(self.click_mode, int(i)))
        top_buttons2 = [None,self.buttonLeft, self.buttonRight, self.buttonLeftRight, self.buttonUp]
        for i in range(1,5):
            top_buttons2[i].clicked.connect(partial(self.click_signal, int(i)))

       

def main():
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
