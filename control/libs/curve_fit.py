import numpy as np
import matplotlib.pyplot as plt
from numpy.polynomial.polynomial import Polynomial

# 주어진 데이터
kph = np.array([50, 60, 70])
lfd = np.array([27, 33, 39])
lfd_offset = np.array([0.80, 0.85])
# 2차 다항식으로 회귀
coeffs = np.polyfit(kph[:-1], lfd_offset, 1)
p = Polynomial(coeffs)

# kph 범위에서 lfd 값을 예측
kph_range = np.linspace(30, 90, 100)
lfd_pred = np.polyval(coeffs, kph_range)

# 그래프 그리기
plt.scatter(kph[:-1], lfd_offset, color='red', label='Data points')
plt.plot(kph_range, lfd_pred, label='Fitted curve', color='blue')
plt.xlabel('kph')
plt.ylabel('LFD')
plt.legend()
plt.title('LFD vs KPH Curve Fitting')
plt.grid(True)
plt.show()

# 회귀 계수 출력
print(f"다항식 계수: {coeffs}")

speed = 40
print(1.500e-03*speed**3 -2.700e-01*speed**2 + 1.635e+01*speed -3.030e+02)