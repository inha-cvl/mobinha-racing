import rospy
import sys
import signal
from geometry_msgs.msg import PoseStamped

def signal_handler(sig, frame):
    sys.exit(0)


class ObstacleSimulator:
    def __init__(self):
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
    
    def goal_cb(self, msg):
        '''
        pose: 
  position: 
    x: -63.725067138671875
    y: 124.38723754882812
    z: 0.0
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.9334764381951742
    w: 0.3586387309458518

        '''    


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('obstacle_simulator', anonymous=False)
    obstacle_simulator = ObstacleSimulator()
    obstacle_simulator.run()

if __name__ == "__main__":
    main()