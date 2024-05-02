from PyQt5.QtWidgets import QWidget, QScrollArea,  QVBoxLayout, QWidget,QTableWidget, QTableWidgetItem


class MessageViewer(QWidget):
    def __init__(self, TH):
        super().__init__()
        self.decode_handler = TH.decode_handler
        self.tables = {}
        self.init_ui()
    
    def init_ui(self):
        self.scroll_area = QScrollArea(self)  # 스크롤 가능한 영역 생성
        self.scroll_widget = QWidget()  # 스크롤 영역에 넣을 위젯
        self.scroll_layout = QVBoxLayout(self.scroll_widget)  # 스크롤 위젯에 레이아웃 추가

        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)

        self.layout = QVBoxLayout(self)
        self.layout.addWidget(self.scroll_area)
        self.setWindowTitle('CAN Messages Viewer')
        self.show()

    def create_table_for_msg_id(self, msg_id):
        table = QTableWidget()
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(['Name', 'Value'])
        self.scroll_layout.addWidget(table)
        self.tables[msg_id] = table

    def update_table(self, msg_id, data_dict):
        if msg_id not in self.tables:
            self.create_table_for_msg_id(msg_id)

        print(data_dict)
        table = self.tables[msg_id]
        table.setRowCount(len(data_dict))  # 데이터 사전 크기에 맞게 행 수 조절

        for index, (key, value) in enumerate(data_dict.items()):
            table.setItem(index, 0, QTableWidgetItem(key))
            table.setItem(index, 1, QTableWidgetItem(str(value)))