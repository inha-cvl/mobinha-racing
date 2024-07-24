#!/usr/bin/env python

class NAVPVT:

    def __init__(self):
        self.condition = False
        self.iTOW = None
        self.year = None
        self.month = None
        self.day = None
        self.hour = None
        self.min = None
        self.sec = None
        self.valid = None
        self.tAcc = None
        self.nano = None
        self.fixType = None
        self.flags = None
        self.flags2 = None
        self.numSV = None
        self.lon = None
        self.lat = None
        self.height = None
        self.hMSL = None
        self.hAcc = None
        self.vAcc = None
        self.velN = None
        self.velE = None
        self.velD = None
        self.gSpeed = None
        self.heading = None
        self.sAcc = None
        self.headAcc = None
        self.pDOP = None
        self.reserved1 = None
        self.headVeh = None
        self.magDec = None
        self.magAcc = None


    def callback(self, msg):
        self.condition = True
        self.iTOW = msg.iTOW
        self.year = msg.year
        self.month = msg.month
        self.day = msg.day
        self.hour = msg.hour
        self.min = msg.min
        self.sec = msg.sec
        self.valid = msg.valid
        self.tAcc = msg.tAcc
        self.nano = msg.nano
        self.fixType = msg.fixType
        self.flags = msg.flags
        self.flags2 = msg.flags2
        self.numSV = msg.numSV
        self.lon = msg.lon
        self.lat = msg.lat
        self.height = msg.height
        self.hMSL = msg.hMSL
        self.hAcc = msg.hAcc
        self.vAcc = msg.vAcc
        self.velN = msg.velN
        self.velE = msg.velE
        self.velD = msg.velD
        self.gSpeed = msg.gSpeed
        self.heading = msg.heading*1e-5
        self.sAcc = msg.sAcc
        self.headAcc = msg.headAcc
        self.pDOP = msg.pDOP
        self.reserved1 = msg.reserved1
        self.headVeh = msg.headVeh
        self.magDec = msg.magDec
        self.magAcc = msg.magAcc
