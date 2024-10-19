## obstacle_info.py
import numpy as np
import rospy
import math
from collections import deque

class Object():
    def __init__(self, bbox):
        self.bbox = bbox
        
        self.prev_x = bbox[0]
        self.prev_y = bbox[1]
        self.x_est = np.array([bbox[0], 0.0])  # [위치 x, 속도 v_x]
        self.y_est = np.array([bbox[1], 0.0])  # [위치 y, 속도 v_y]

        self.x_velocities = deque(maxlen=10)
        self.mean_velocity_x = 0.0


        self.z_x = 0.0
        self.z_y = 0.0
        self.collision_pos_y = 0.0
        self.appear = 0
        self.z_score = 2.58 # [1.645, 1.96, 2.58]

    def set_initial_value(self, z_x, z_y):
        self.x_est = np.array([z_x, 0.0])
        self.y_est = np.array([z_y, 0.0])
        self.prev_x = z_x
        self.prev_y = z_y
        self.appear += 1

    def predict(self, dt):
        self.prev_x = self.x_est[0] + self.x_est[1]*dt
        self.prev_y = self.y_est[0] + self.y_est[1]*dt

    def update(self, z_x, z_y, dt):
        self.x_est[0] = z_x
        self.y_est[0] = z_y
        if dt > 0:
            self.x_est[1] = (self.x_est[0] - self.prev_x) / dt
            self.y_est[1] = (self.y_est[0] - self.prev_y) / dt
        else:
            self.x_est[1] = 0.0
            self.y_est[1] = 0.0
        
        v_x = self.x_est[1]
        v_y = self.y_est[1]
        velocity = np.sqrt(v_x*v_x + v_y*v_y)
        
        self.x_velocities.append(v_x)
        # self.mean_velocity_x = np.mean(self.x_velocities)
        if len(self.x_velocities) >= 3:
            mean_x = np.mean(self.x_velocities)
            std_x = np.std(self.x_velocities)

            threshold = self.z_score* std_x

            if abs(v_x - mean_x) <= threshold:
                self.x_velocities.append(v_x)
            
        else:
            self.x_velocities.append(v_x)
        
        self.mean_velocity_x = np.mean(self.x_velocities)
        # if v_x >= self.mean_velocity_x:
        #     if (v_x - self.mean_velocity_x)/dt <= 2.0:
        #         self.x_velocities.append(v_x)
        #         self.mean_velocity_x = np.mean(self.x_velocities)
        #     else:
        #         print("velocity miss calculated !! ignore that value")
        # else:
        #     self.x_velocities.append(v_x)
        #     self.mean_velocity_x = np.mean(self.x_velocities)

        if velocity >= 90:    
            print("velocity : ", round(velocity, 2))
        self.prev_x = self.x_est[0]
        self.prev_y = self.y_est[0]
        self.appear += 1


