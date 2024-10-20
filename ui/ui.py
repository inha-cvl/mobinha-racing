import sys
import signal
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QTimer
from datetime import datetime
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
        self.kiapi_signal_buttons = [None, self.buttonGo, self.buttonStop, self.buttonSlowOn, self.buttonSlowOff, self.buttonPitStop]
        self.kiapi_signal_strings = ['WAIT', 'GO', 'STOP', 'SLOW ON', 'SLOW OFF', 'PIT STOP']
        self.prev_kiapi_signal = 0
        self.prev_system_mode = 0
        self.start_time = None

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
        self.timer.start(500)

        self.user_input_timer = QTimer(self)
        self.user_input_timer.timeout.connect(self.RH.publish)
        self.user_input_timer.start(1000)


    def updateUI(self):
        self.speedometer_widget.set_speed(self.RH.ego_value['velocity'], self.RH.target_value['velocity'])
        self.speed_graph.set_speed(self.RH.ego_value['velocity'], self.RH.target_value['velocity'])
        self.steer_graph.set_speed(self.RH.ego_value['steer'], self.RH.target_value['steer'])
        self.wheel_widget.set_yaw(self.RH.ego_value['steer'], self.RH.target_value['steer'])
        self.gear_widget.set_gear(self.RH.ego_value['gear'])

        self.system_label_update(self.RH.system_status['mode'], self.RH.system_status['kiapi_signal'],              
                                 self.RH.lane_number, self.RH.system_status['lap_count'], self.RH.planning_mode)

        self.can_table_update(self.RH.can_inform)
        self.kiapi_signal_update(self.RH.system_status['kiapi_signal'])
        self.update_cumulative_time()

        
    def system_label_update(self, mode, signal, lane_number, lap_count, planning_mode):
        self.systemLabel1.setText(self.mode_strings[int(mode)])

        
        if self.prev_system_mode != int(mode):
            if int(mode)==1:
                self.systemLabel1.setStyleSheet("""QLabel {background-color: rgb(4,188,124); border-radius: 5px; color: rgb(255,255,255)}""")
            elif int(mode) == 2 or int(mode) == 3:
                self.systemLabel1.setStyleSheet("""QLabel {background-color: rgb(252,203,28); border-radius: 5px; color: rgb(255,255,255)}""")
            elif int(mode) == 0:
                self.systemLabel1.setStyleSheet("""QLabel {background-color: rgb(252,36,68); border-radius: 5px; color: rgb(255,255,255)}""")
            self.prev_system_mode = int(mode)
        
        if self.prev_kiapi_signal != int(signal):
            self.systemLabel2.setText(self.kiapi_signal_strings[int(signal)])
            if int(signal) == 1:
                self.systemLabel2.setStyleSheet("""QLabel {background-color:  rgb(0, 102, 255); border-radius: 5px; color: black;}""")
            elif int(signal) == 2:
                self.systemLabel2.setStyleSheet("""QLabel {background-color:  rgb(252,36,68); border-radius: 5px; color: black;}""")
            elif int(signal) == 5:
                self.systemLabel2.setStyleSheet("""QLabel {background-color:  rgb(246, 126, 82); border-radius: 5px; color: black;}""")
            else:
                self.systemLabel2.setStyleSheet("""QLabel {background-color: rgb(252,203,28); border-radius: 5px; color: black;}""")
       
        self.laneNumberLabel.setText(f"Lane: {lane_number}")
        self.planningModeLabel.setText(f"{planning_mode}")
        self.lapCountLabel.setText(f"Lap: {lap_count}")
        
        
    def update_cumulative_time(self):
        if self.start_time is not None:
            time_diff = datetime.datetime.now() - self.start_time
            hours = time_diff.seconds // 3600
            minutes = (time_diff.seconds % 3600) // 60
            seconds = time_diff.seconds % 60
            milliseconds = time_diff.microseconds // 1000

            formatted_time = f"{hours:02}:{minutes:02}:{seconds:02}:{milliseconds:03}"

            self.timeCountLabel.setText(formatted_time)
    
    def can_table_update(self, can_inform):
        self.canTable.setItem(0, 1, QTableWidgetItem(can_inform['eps_status']))
        self.canTable.setItem(1, 1, QTableWidgetItem(can_inform['acc_status']))

    def kiapi_signal_update(self, kiapi_signal):
        if kiapi_signal == 0:
            return
        idx = kiapi_signal
        if self.prev_kiapi_signal != idx:
            for i, kiapi_button in enumerate(self.kiapi_signal_buttons):
                if i == 0:
                    continue 
                if i == idx:
                    if i == 1:
                        self.start_time = datetime.datetime.now()
                        kiapi_button.setStyleSheet(""" QPushButton {background-color: #0066ff;color: black;}""")
                    elif i == 2:
                        kiapi_button.setStyleSheet(""" QPushButton {background-color: rgb(252,36,68);color: black;}""")
                    elif i == 5:
                        kiapi_button.setStyleSheet(""" QPushButton {background-color: rgb(246, 126, 82);color: black;}""")
                    else:
                        kiapi_button.setStyleSheet(""" QPushButton {background-color: rgb(252,203,28);color: black;}""")
                else:
                    kiapi_button.setStyleSheet(""" QPushButton {background-color: #eeeeec; color: black;}""")
             

    def click_mode(self, mode):
        self.RH.user_value['user_mode'] = mode
        self.check_timer()
    
    def click_signal(self, v):
        self.RH.user_value['user_signal'] = v
        self.check_timer()
    
    def click_reset(self):
        self.RH.user_value['user_signal'] = 5
        self.check_timer()

    # KIAPI CAN signal
    def click_kiapi_signal(self, signal_value):
        self.RH.user_value['kiapi_signal'] = signal_value
        self.check_timer()

    def check_timer(self):
        if not self.sig_in:
            self.sig_in = True
            self.user_input_timer.start(500)
            QTimer.singleShot(5000, self.stop_user_input_timer)
        else:
            self.stop_user_input_timer
    
    def stop_user_input_timer(self):
        self.sig_in = False
        self.RH.user_value['user_signal'] = 0
        self.RH.user_value['kiapi_signal'] = 0
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
        top_buttons2 = [None, self.buttonLeftRight, self.buttonLeft, self.buttonRight ]
        for i in range(1,4):
            top_buttons2[i].clicked.connect(partial(self.click_signal, int(i)))
        for i in range(1,6):
            self.kiapi_signal_buttons[i].clicked.connect(partial(self.click_kiapi_signal, int(i)))
        self.resetButton.clicked.connect(self.click_reset)

def main():
    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    signal.signal(signal.SIGINT, signal_handler)
    app.exec_()

if __name__ == '__main__':
    main()
