import rospy

import sys
import signal

from ros_handler import ROSHandler
from libs.apid import APID
# from libs.purepursuit import PurePursuit, PurePursuit_test

import rospy


import pymap3d
import numpy as np


def signal_handler(sig, frame):
    sys.exit(0)

class Control():
    def __init__(self):
        self.RH = ROSHandler()
        self.APID = APID(self.RH)

        # self.PP = PurePursuit(self.RH)

        # geo_path = []
        # with open('/home/jourmain/mobinha/selfdrive/car/test/libs/path_log.txt', 'r') as file:
        # # with open('path_log.txt', 'r') as file:
        #     lines = file.readlines() 

        # for line in lines:
        #     stripped_line = line.strip()
        #     lat, long = stripped_line.split(',')
        #     geo_path.append((float(lat), float(long)))
        # base_lat = geo_path[0][0]
        # base_lon = geo_path[0][1]

        # self.path = []
        # for i in range(len(geo_path)):
        #     x, y, _ = pymap3d.geodetic2enu(
        #     geo_path[i][0], geo_path[i][1], 0, base_lat, base_lon, 0)
        #     self.path.append((x,y))

        # self.PP = PurePursuit_test(self.RH, self.path)

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            acc = self.APID.execute()
            # steer = self.PP.execute()
            steer = 0
            self.RH.publish(acc, steer)

            # self.cte = self.calculate_cte(self.RH.current_location)
            # print("CTE", self.cte)
            rate.sleep()

    # def calculate_cte(self, position):
    #     idx = self.calc_idx(position)
    #     self.idx = idx
    #     Ax, Ay = self.path[idx]
    #     Bx, By = self.path[idx+1]
    #     Px, Py = position

    #     numerator = abs((Bx - Ax) * (Ay - Py) - (Ax - Px) * (By - Ay))
    #     denominator = np.sqrt((Bx - Ax)**2 + (By - Ay)**2)
    #     cte = numerator / denominator if denominator != 0 else 0

    #     cross_product = (Bx - Ax) * (Py - Ay) - (By - Ay) * (Px - Ax)
        
    #     if cross_product > 0:
    #         return -cte
    #     elif cross_product < 0:
    #         return cte
    #     else:
    #         return 0
        
        
    # def calc_idx(self, pt):
    #     min_dist = float('inf')
    #     min_idx = 0

    #     for idx, pt1 in enumerate(self.path):
    #         dist = np.sqrt((pt[0]-pt1[0])**2+(pt[1]-pt1[1])**2)
    #         if dist < min_dist:
    #             min_dist = dist
    #             min_idx = idx

    #     if min_idx == len(self.path) - 1:
    #         pt1 = self.path[min_idx-1]
    #     else:
    #         pt1 = self.path[min_idx]

    #     return min_idx

def main():
    signal.signal(signal.SIGINT, signal_handler)
    control = Control()
    control.execute()

if __name__ == "__main__":
    main()