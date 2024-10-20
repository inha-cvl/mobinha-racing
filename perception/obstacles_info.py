import numpy as np
from collections import deque

class Object:
    def __init__(self, bbox):
        self.bbox = bbox
        self.dt = 0.1  # 시간 간격

        # 상태 벡터: [x, y, vx, vy]
        self.state_est = np.array([bbox[0], bbox[1], 0.0, 0.0])

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        # my ye chuk noise
        self.Q = np.eye(4) * 0.1

        # sensor noise(detection noise)
        self.R = np.eye(2) * 0.005

        # initial velocity little bit bu-jeunghwak
        self.P_est = np.diag([1.0, 1.0, 10.0, 10.0])

        self.count = 0
        self.appear = 0
        

    def set_initial_value(self, z_x, z_y):
        self.state_est = np.array([z_x, z_y, 0.0, 0.0])
        self.count += 1

    def predict(self, dt):

        F = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.state_pred = F.dot(self.state_est)
        self.P_pred = F.dot(self.P_est).dot(F.T) + self.Q

    def update(self, z_x, z_y, dt):

        self.predict(dt)

        z = np.array([z_x, z_y])

        S = self.H.dot(self.P_pred).dot(self.H.T) + self.R
        K = self.P_pred.dot(self.H.T).dot(np.linalg.inv(S))


        y = z - self.H.dot(self.state_pred)  
        # print("residual : ", y)
        # print("kalman gain : ", K[0][0], K[1][1])
        self.state_est = self.state_pred + K.dot(y)
        self.P_est = (np.eye(4) - K.dot(self.H)).dot(self.P_pred)

        self.appear += 1

    def future_point(self):

        forward = []
        for i in range(5):
            dt_future = self.dt * (i + 1)
            F_future = np.array([
                [1.0, 0.0, dt_future, 0.0],
                [0.0, 1.0, 0.0, dt_future],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            future_state = F_future.dot(self.state_est)
            forward.append([future_state[0], future_state[1]])
        return forward
