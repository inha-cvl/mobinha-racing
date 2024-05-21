from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QScrollArea,  QVBoxLayout,QHBoxLayout,  QWidget,QTableWidget, QTableWidgetItem, QLabel

import rospy
import sys
import signal

from drive_msgs.msg import *
from transmitter_handler import TransmitterHandler


def signal_handler(sig, frame):
    sys.exit(0)

class MessageViewer(QWidget):
    def __init__(self):
        super().__init__()
        rospy.init_node("can_viewer")
        self.TH = TransmitterHandler()
        self.tables = {}
        self.init_ui()
        self.set_protocol()
    
    def set_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/CANInput', CANInput, self.can_input_cb)
        
    
    def init_ui(self):
        self.setGeometry(800,1000,800,1000)
        self.setWindowTitle('Message Viewer')

        self.scroll_area1 = QScrollArea(self)  # 스크롤 가능한 영역 생성
        self.scroll_widget1 = QWidget()  # 스크롤 영역에 넣을 위젯
        self.scroll_layout1 = QVBoxLayout(self.scroll_widget1)  # 스크롤 위젯에 레이아웃 추가
        self.scroll_area1.setWidgetResizable(True)
        self.scroll_area1.setWidget(self.scroll_widget1)

        self.scroll_area2 = QScrollArea(self)  # 스크롤 가능한 영역 생성
        self.scroll_widget2 = QWidget()  # 스크롤 영역에 넣을 위젯
        self.scroll_layout2 = QVBoxLayout(self.scroll_widget2)  # 스크롤 위젯에 레이아웃 추가
        self.scroll_area2.setWidgetResizable(True)
        self.scroll_area2.setWidget(self.scroll_widget2)
        

        self.central_widget = QWidget(self)
        self.central_layout = QHBoxLayout(self)
        self.layout1 = QVBoxLayout()
        self.layout1.addWidget(self.scroll_area1)
        self.layout2 = QVBoxLayout()
        self.layout2.addWidget(self.scroll_area2)
        self.central_layout.addLayout(self.layout1)
        self.central_layout.addLayout(self.layout2)
        self.set_tables()
        self.show()

    def set_tables(self):
        for key in self.TH.encode_handler.keys():
            self.create_table_for_msg_id_en(key)
        for key in self.TH.decode_handler.keys():
            self.create_table_for_msg_id_de(key)
        
    def create_table_for_msg_id_en(self, msg_id):
        table = QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(['Name', 'Value'])
        label = QLabel(str(hex(msg_id)))
        self.scroll_layout1.addWidget(label)
        self.scroll_layout1.addWidget(table)
        self.tables[msg_id] = table
    
    def create_table_for_msg_id_de(self, msg_id):
        table = QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(['Name', 'Value'])
        label = QLabel(str(hex(msg_id)))
        self.scroll_layout2.addWidget(label)
        self.scroll_layout2.addWidget(table)
        self.tables[msg_id] = table


    def update_table(self, msg_id, data_dict):
        table = self.tables[msg_id]
        table.setRowCount(len(data_dict))  # 데이터 사전 크기에 맞게 행 수 조절
        for index, (key, value) in enumerate(data_dict.items()):
            table.setItem(index, 0, QTableWidgetItem(key))
            table.setItem(index, 1, QTableWidgetItem(str(value)))

    def can_output_cb(self, msg):
        for keys, values in self.TH.decode_handler.items():
            for key in values.keys():
                values[key] = getattr(msg, key).data
            self.update_table(keys, values)
    
    def can_input_cb(self, msg):
        for keys, values in self.TH.encode_handler.items():
            for key in values.keys():
                values[key] = getattr(msg, key).data
            self.update_table(keys, values)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    app = QApplication(sys.argv)
    message_viewer = MessageViewer()
    app.exec_()