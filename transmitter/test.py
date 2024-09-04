import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np

class Visualizer:
    def __init__(self):
        # 초기 객체 설정 (테스트용 데이터로 초기화)
        self.setup_message_dicts()
        self.fig, self.ax = plt.subplots(figsize=(7, 12))  # 플롯 설정

    def setup_message_dicts(self):
        # 기본 RDR_Obj 딕셔너리 설정
        base_RDR_Obj = {
            'RelPosX01': 0.05,
            'RelPosY01': 0.05,
            'RelPosX02': 0.05,
            'RelPosY02': 0.05,
        }

        for i in range(1, 17):  # RDR_Obj_01 ~ RDR_Obj_16 설정
            setattr(self, f'RDR_Obj_{i:02}', base_RDR_Obj.copy())

    # 가상의 데이터를 생성하는 함수 (테스트용)
    def update_fake_data(self):
        for i in range(1, 17):
            obj = getattr(self, f'RDR_Obj_{i:02}')
            obj['RelPosX01'] = np.random.uniform(0, 205)
            obj['RelPosY01'] = np.random.uniform(-103, 103)
            obj['RelPosX02'] = np.random.uniform(0, 205)
            obj['RelPosY02'] = np.random.uniform(-103, 103)

    def init_plot(self):
        # 초기 플롯 설정
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(0, 205)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='white')
        return []

    # 장애물 표시 함수 (매 프레임 업데이트)
    def update_plot(self, frame):
        self.ax.cla()  # 플롯을 초기화
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(0, 205)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='white')

        # 데이터 갱신
        self.update_fake_data()

        # 장애물 그리기
        patches_list = []
        for i in range(1, 17):
            obj = getattr(self, f'RDR_Obj_{i:02}')
            x1, y1 = obj['RelPosX01'], obj['RelPosY01']
            x2, y2 = obj['RelPosX02'], obj['RelPosY02']

            # 네모 상자 그리기 (x, y 반대로 그리기) - cyan 색상과 테두리 설정
            rect1 = patches.Rectangle((y1-1, x1-1), 2, 3.5, linewidth=1.5, edgecolor='cyan', facecolor='none')
            rect2 = patches.Rectangle((y2-1, x2-1), 2, 3.5, linewidth=1.5, edgecolor='cyan', facecolor='none')
            self.ax.add_patch(rect1)
            self.ax.add_patch(rect2)
            patches_list.extend([rect1, rect2])

        # 눈금 및 축 설정
        self.ax.set_xticks(np.arange(-50, 50, 3))
        self.ax.set_yticks(np.arange(0, 206, 10))
        self.ax.set_xlabel("Lateral (m)", fontsize=9)
        self.ax.set_ylabel("Longitudinal (m)", fontsize=9)
        self.ax.tick_params(labelsize=7)
        plt.subplots_adjust(left=0.08, right=0.99, top=0.99, bottom=0.05)

        return patches_list

    # 애니메이션 실행
    def run(self):
        ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, frames=np.arange(0, 100, 1), interval=100)
        plt.show()

# 인스턴스 생성 및 실행
visualizer = Visualizer()
visualizer.run()
