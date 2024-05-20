import numpy as np
import matplotlib.pyplot as plt

# 파라미터 설정
total_time = 300  # 전체 시간 (초)
accel_decel_time = 7  # 가속 및 감속에 걸리는 시간 (초)
peak_time = 3.5  # 가속도가 최대가 되는 시점 (초)

# 시간 배열 생성
t = np.linspace(0, total_time, 1000)  # 0부터 300초까지 1000개의 점으로 나누기

# 가속도 계산
def calculate_acceleration(t, total_time, accel_decel_time, peak_time):
    # 가속 구간에서의 가속도
    if t < accel_decel_time:
        if t < peak_time:
            return (t) / peak_time
        else:
            return 1 - (t) / peak_time
    # 감속 구간에서의 가속도
    elif t > total_time - accel_decel_time:
        t_rel = t - (total_time - accel_decel_time)
        if t_rel < peak_time:
            return -(t_rel) / peak_time
        else:
            return -(1 - (t_rel) / peak_time)
    # 등속 구간
    else:
        return 0

# 가속도 프로파일 계산
acceleration = [calculate_acceleration(ti, total_time, accel_decel_time, peak_time) for ti in t]

# 속도 프로파일 계산 (가속도 적분)
velocity = np.cumsum(acceleration) * (t[1] - t[0])

# 속도 그래프 그리기
plt.plot(t, acceleration)
plt.show()


total_time = int((len(final_path) / max_vel) + (sec_to_reach*2))
t = np.linspace(0, total_time, total_time * 10)  # 시간 배열

plt.figure(figsize=(10, 15))  # 전체 그래프 크기 조정

# 가속도 그래프
plt.subplot(3, 1, 1)  # 3행 1열의 첫 번째 위치에 그래프 배치
plt.plot(t, accel_p, label="Acceleration")
plt.title("Acceleration Profile")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
plt.xlim([0,14])
plt.ylim([0,10])

# 속도 그래프
plt.subplot(3, 1, 2)  # 3행 1열의 첫 번째 위치에 그래프 배치
plt.plot(t, vel_p, label="Velocity", color='orange')
plt.title("Velocity Profile")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.xlim([0,14])
plt.ylim([0,30])

# rjfl 그래프
plt.subplot(3, 1, 3)  # 3행 1열의 첫 번째 위치에 그래프 배치
plt.plot(t, dist_p, label="Disatnace", color='green')
plt.title("Distance Profile")
plt.xlabel("Time (s)")
plt.ylabel("Distance")
plt.xlim([0,14])
plt.ylim([0,500])

plt.tight_layout()  # 서브플롯 간격 자동 조정
plt.show()