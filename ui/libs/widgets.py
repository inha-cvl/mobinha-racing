from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtGui import QPainter, QPen, QColor, QTransform, QFont, QBrush, QPixmap, QLinearGradient, QGradient
from PyQt5.QtCore import Qt, QPointF
from rviz import bindings as rviz
import math
import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class RvizWidget(rviz.VisualizationFrame):
    def __init__(self, parent=None):
        super(RvizWidget, self).__init__(parent)
        self.setContentsMargins(0, 0, 0, 0)
        self.setSplashPath('')
        self.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "./rviz/control_ui.rviz")
        self.load(config)
        self.setMenuBar(None)
        self.setStatusBar(None)

class SpeedometerWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.speed = 0.0
        self.target_speed = 0.0
        self.setFixedWidth(150)
        self.setContentsMargins(0, 0, 0, 0)

    def set_speed(self, e,t):
        self.speed = e
        self.target_speed = t
        self.update()
    

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        major_ticks = list(range(210, -31, -40))  
        minor_ticks = list(range(210, -31, -10)) 

        for angle in major_ticks:
            self.draw_tick(painter, angle, major=True)
            self.draw_label(painter, angle) 

        for angle in minor_ticks:
            if angle not in major_ticks:
                self.draw_tick(painter, angle, major=False)

        self.draw_needle(painter, self.target_speed,QColor('#ff24a8'))
        self.draw_needle(painter, self.speed, QColor('#005eff'))

        painter.setTransform(QTransform()) 
        painter.setPen(QPen(Qt.black)) 
        painter.setFont(QFont('Arial', 16))
        painter.drawText(self.rect().center()+QPointF(-30,90), f"{self.speed}\nkm/h")
    

    def draw_needle(self, painter, speed, color):
        needle_length = min(self.width(), self.height()) / 3.5
        needle_angle = 210 - speed
        if needle_angle <= 0:
            needle_angle = -needle_angle
        needle_angle -= 90
        needle_angle = 360 - needle_angle
        needle_pivot = self.rect().center()
        painter.setPen(QPen(color, 5))
        transform = QTransform().translate(needle_pivot.x(), needle_pivot.y()).rotate(needle_angle).translate(-needle_pivot.x(), -needle_pivot.y())
        painter.setTransform(transform)
        painter.drawLine(needle_pivot, needle_pivot - QPointF(0, needle_length))

    def draw_tick(self, painter, angle, major=True):
        tick_length = 10 if major else 5
        tick_width = 2

        center = self.rect().center()
        radius = min(self.width(), self.height()) / 2.0 - 30  

        start_point = center + QPointF(radius * math.cos(math.radians(angle)), -radius * math.sin(math.radians(angle)))
        end_point = start_point + QPointF(tick_length * math.cos(math.radians(angle)), -tick_length * math.sin(math.radians(angle)))

        painter.setPen(QPen(Qt.black, tick_width))
        painter.drawLine(start_point, end_point)
        

    def draw_label(self, painter, angle):
        if angle-210 <= 0:
            value = -(angle-210)
        else:
            value = angle

        center = self.rect().center()
        radius = min(self.width(), self.height())/2.0 - 20 
        label_radius = radius+7

        # Position for the label text
        text_position = center + QPointF(label_radius * math.cos(math.radians(angle)), -label_radius * math.sin(math.radians(angle)))

        painter.setPen(QPen(Qt.black))
        painter.setFont(QFont('Arial', 10))
        painter.drawText(text_position, str(value))

class WheelWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = 0.0
        self.target_yaw = 0.0
        self.setFixedWidth(150)
        self.setContentsMargins(0, 0, 0, 0)

    def set_yaw(self, e,t):
        self.yaw = e
        self.target_yaw = t
        self.update()


    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        wheel_radius = min(self.width(), self.height())/2-30
        wheel_center = self.rect().center()

        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawEllipse(wheel_center, wheel_radius, wheel_radius)

        self.draw_line(painter, wheel_center, self.target_yaw,  QColor('#ff24a8'))
        self.draw_line(painter, wheel_center, self.yaw, QColor('#005eff'))

        painter.setPen(QPen(Qt.black))
        painter.setFont(QFont('Arial', 16))
        painter.drawText(wheel_center+QPointF(-30, 90), f"{self.yaw:.2f}\ndeg")

    def draw_line(self, painter, center, angle, color):
        if angle < 0:
            dg = 90-angle
        else:
            dg = 90-angle

        line_length = min(self.width(), self.height()) / 3.5
        line_end = center + QPointF(line_length * math.cos(math.radians(-dg)),
                                    line_length * math.sin(math.radians(-dg)))

        painter.setPen(QPen(color, 5))
        painter.drawLine(center, line_end)


class GaugeWidget(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.value = 0
        self.v = 0
        self.target = 0
        self.t = 0
        self.gauge_label = QLabel(self)
        self.gauge_label.setAlignment(Qt.AlignCenter)
        self.setContentsMargins(0, 0, 0, 0)
        self.update_gauge()
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.gauge_label)
        label = QLabel(self)
        label.setFixedHeight(20)
        label.setAlignment(Qt.AlignCenter)
        label.setText(title)
        layout.addWidget(label)
        self.setLayout(layout)
    
    def update_gauge(self):
        gradient = QLinearGradient(0, 0, 0, 50)
        gradient.setColorAt(0, QColor(255, 0, 0, max(30, int(255 * (self.value / 100)))))  # 빨간색 (시작)
        gradient.setColorAt(1, QColor(255, 255, 0, int(255 * (self.value / 100))))  # 노란색 (끝)
        brush = QBrush(gradient)
        pixmap = self.draw_gauge(brush)
        self.gauge_label.setPixmap(pixmap)

    def draw_gauge(self, brush):
        pixmap = self.create_gauge_pixmap(100, 50, brush)
        return pixmap

    def create_gauge_pixmap(self, width, height, brush):
        pixmap = QPixmap(width, height)
        pixmap.fill(Qt.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        mini_height = 185

        gauge_rect = pixmap.rect()
        fill_height = int((self.value / 100) * mini_height)
        pos = gauge_rect.height()-fill_height
        fill_rect = gauge_rect.translated(0, pos)

        
        painter.setBrush(brush)
        painter.drawRect(fill_rect)

        # Draw the target triangle
        target_pos = gauge_rect.height() - int((self.target / 100) * mini_height)
        target_triangle = [QPointF(0, target_pos), QPointF(10, target_pos-10), QPointF(10, target_pos)]    
        painter.setBrush(QColor('#005eff'))
        painter.drawPolygon(*target_triangle)


        painter.setFont(QFont('Arial', 10))
        painter.drawText(QPointF(75, pos-3), f"{self.v:.2f}")
        painter.drawText(QPointF(15, target_pos-3), f"{self.t:.2f}")

        painter.end()

        return pixmap

    def set_value(self, value):
        self.v = value
        self.value = max(0, min(100, value))
        self.update_gauge()
    
    def set_target(self, value):
        self.t = value
        self.target= max(0, min(100, value))
        self.update_gauge()

class LiveSpeedGraph(FigureCanvas):
    def __init__(self, type, parent=None):
        fig = Figure()
        self.axes = fig.add_subplot(111)  # 1x1 그리드의 첫 번째 subplot
        super(LiveSpeedGraph, self).__init__(fig)
        self.current_speeds = []
        self.target_speeds = []
        self.times = []

        self.axes.set_xlim(0, 10)  # 5초 동안의 데이터를 보여줍니다.
        if str(type) == 'speed':
            self.axes.set_ylim(0, 120)  # 속도의 범위를 0에서 120까지로 가정합니다.
        else:
            self.axes.set_ylim(-300, 300)

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
        self.axes.plot(self.times, self.current_speeds, label='Current', color='#005eff')
        self.axes.plot(self.times, self.target_speeds, label='Target',  color='#ff24a8')
        self.axes.set_xlim(self.times[0], max(10, self.times[-1]))  # x축 범위를 동적으로 조정
        self.axes.legend()
        self.draw()


class SpeedSubscriberWidget(QWidget):
    def __init__(self, type, parent=None):
        super().__init__(parent)
        self.current_speed = 0
        self.target_speed = 0
        self.initUI(type)

    def set_speed(self, e, t):
        self.current_speed = e
        self.target_speed = t
        self.update_graph()

    def initUI(self, type):
        self.setContentsMargins(0, 0, 0, 0)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        self.graph = LiveSpeedGraph(type,self)
        layout.addWidget(self.graph)
        self.setLayout(layout)
        self.setWindowTitle('ROS Speed Graph')
        self.setGeometry(100, 100, 800, 600)

    def update_graph(self):
        self.graph.update_graph(self.current_speed, self.target_speed)


class GearWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.gears = ["P", "R", "N", "D"]
        self.labels = {}

        self.initUI()

    def initUI(self):
        self.setContentsMargins(0, 0, 0, 0)
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        for gear in self.gears:
            label = QLabel(gear, self)
            label.setFixedSize(40, 40)
            label.setAlignment(Qt.AlignCenter)
            self.labels[gear] = label
            layout.addWidget(label)

        self.set_gear("P")

    def set_gear(self, gear):
        for g, label in self.labels.items():
            if g == gear:
                self.set_label_color(label, "#ff69b4")  # Pink
            else:
                self.set_label_color(label, "#d3d3d3")  # Light gray

    def set_label_color(self, label, color):
        label.setStyleSheet(f"""
            background-color: {color};
            color: white;
            border-radius: 10px;
            border: 2px solid {color};
        """)