from PyQt5.QtWidgets import QWidget, QVBoxLayout
import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class LiveSpeedGraph(FigureCanvas):
    def __init__(self, parent=None):
        fig = Figure()
        self.axes = fig.add_subplot(111)  # 1x1 그리드의 첫 번째 subplot
        super(LiveSpeedGraph, self).__init__(fig)
        self.current_speeds = []
        self.target_speeds = []
        self.times = []

        self.axes.set_xlim(0, 10)  # 5초 동안의 데이터를 보여줍니다.
        self.axes.set_ylim(0, 120)  # 속도의 범위를 0에서 120까지로 가정합니다.

    def update_graph(self, current_speed, target_speed):
        now = datetime.datetime.now()
        # 최신 시간 데이터를 추가합니다.
        if self.times:
            # 현재 시간과의 차이를 계산하여 새로운 시간을 추가합니다.
            new_time = (now - self.reference_time).total_seconds()  # reference_time은 최초 데이터 포인트의 시간입니다.
            self.times.append(new_time)
        else:
            # 첫 번째 데이터 포인트인 경우, 시간 리스트를 초기화합니다.
            self.reference_time = now
            self.times.append(0)

        self.current_speeds.append(current_speed)
        self.target_speeds.append(target_speed)

        # 5초 이상의 오래된 데이터를 제거합니다.
        while self.times and self.times[0] < self.times[-1] - 10:
            self.times.pop(0)
            self.current_speeds.pop(0)
            self.target_speeds.pop(0)

        # 그래프를 다시 그립니다.
        self.axes.clear()
        self.axes.plot(self.times, self.current_speeds, label='Current Speed')
        self.axes.plot(self.times, self.target_speeds, label='Target Speed')
        self.axes.set_xlim(self.times[0], max(10, self.times[-1]))  # x축 범위를 동적으로 조정
        self.axes.legend()
        self.draw()


class VelocityGraph(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_speed = 0
        self.target_speed = 0
        self.initUI()


    def set_speed(self, e,t):
        self.current_speed = e
        self.target_speed = t
        self.update_graph()
    

    def initUI(self):
        layout = QVBoxLayout()
        self.graph = LiveSpeedGraph(self)
        layout.addWidget(self.graph)
        self.setLayout(layout)
        self.setWindowTitle('ROS Speed Graph')
        self.setGeometry(100, 100, 800, 600)


    def update_graph(self):
        self.graph.update_graph(self.current_speed, self.target_speed)
