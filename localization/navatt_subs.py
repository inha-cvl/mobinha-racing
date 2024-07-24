#!/usr/bin/env python
class NAVATT:

    def __init__(self):
        self.condition = False
        self.iTOW = None
        self.version = None
        self.reserved0 = None
        self.roll = None
        self.pitch = None
        self.heading = None
        self.accRoll = None
        self.accPitch = None
        self.accHeading = None

    def callback(self, msg):
        self.condition =True
        self.iTOW = msg.iTOW
        self.version = msg.version
        self.reserved0 = msg.reserved0
        self.roll = msg.roll*1e-5
        self.pitch = msg.pitch*1e-5
        self.heading = msg.heading*1e-5
        self.accRoll = msg.accRoll
        self.accPitch = msg.accPitch
        self.accHeading = msg.accHeading
        # print(f"roll : {self.roll}\npitch : {self.pitch}\nheading : {self.heading}")
        # print("---------------------------------------------------")