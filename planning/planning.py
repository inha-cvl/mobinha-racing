import rospy
import sys
import signal

import cProfile
import pstats
import io
import time

from ros_handler import ROSHandler


#from local_path.local_path_test import LocalPathTest
from local_path.ltpl import LTPL
from longitudinal.adaptive_cruise_control import AdaptiveCruiseControl
from hd_map.map import MAP

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self, map):
        self.map = MAP(map)
        self.RH = ROSHandler(self.map.base_lla)
        #self.lpt = LocalPathTest(self.RH, self.map)
        self.lptl = LTPL(self.RH)
        self.acc = AdaptiveCruiseControl(self.RH)
        self.lp_algorithm = 'LTPL'

    def map_publish(self):
        lmap_viz, mlmap_viz = self.map.get_vizs()
        self.RH.publish_map_viz(lmap_viz, mlmap_viz)

    def execute(self):
        self.map_publish()
        rate = rospy.Rate(20)
        start_time = time.time()
        while not rospy.is_shutdown():            
            if self.lp_algorithm == 'Test':
                lp_result = self.lpt.execute(self.RH.local_pos)
                if lp_result == None:
                    local_path, local_kappa = None, None 
                else:
                    [ local_path, local_kappa ] = lp_result
                local_velocity = self.acc.execute(self.RH.local_pos, local_path, local_kappa)
                self.RH.publish(local_path, local_kappa, local_velocity)
                
            elif self.lp_algorithm == 'LTPL':
                local_action_set = self.lptl.execute(self.RH.local_pos)
                self.RH.publish2(local_action_set)
            
            if time.time() - start_time > 30:
                break
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map = sys.argv[1]
    planning = Planning(map)
    planning.execute()

if __name__ == "__main__":
    # main()

    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()

    # 프로파일링 결과를 문자열로 저장
    stream = io.StringIO()
    stats = pstats.Stats(profiler, stream=stream).sort_stats('cumtime')
    stats.print_stats()

    with open('./profile_output.txt', 'w') as f:
        stats = pstats.Stats(profiler, stream=f).sort_stats('cumtime')
        stats.print_stats()

    print("프로파일링 결과가 profile_output.txt 파일에 저장되었습니다.")