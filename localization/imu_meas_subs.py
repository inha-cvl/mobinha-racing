#!/usr/bin/env python
class IMUMEAS:

    def __init__(self):
        self.header = None
        self.orientation = None
        self.orientation_covariance = None
        self.angular_velocity = None
        self.angular_velocity_covariance = None
        self.linear_acceleration = None
        self.linear_acceleration_covaiance = None

    def callback(self, msg):
        self.header = msg.header
        self.orientation = msg.orientation
        self.orientation_covariance = msg.orientation_covariance
        self.angular_velocity = msg.angular_velocity
        self.angular_velocity_covariance = msg.angular_velocity_covariance
        self.linear_acceleration = msg.linear_acceleration
        self.linear_acceleration_covaiance = msg.linear_acceleration_covariance
