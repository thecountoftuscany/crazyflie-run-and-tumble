import rospy
import rospy_crazyflie.crazyflie_client as crazyflie_client
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
import time
from collections import deque



num_samples = 50

class bcfController:
    def __init__(self):
        # Init node
        rospy.init_node('bcf_Controller')

        # Create deque for calculating average intensity over last num_samples
        self.d_intensity = deque(maxlen=num_samples)

        self.x = 0
        self.y = 0
        self.z = 0
        self.rangeLeft = 0
        self.rangeRight = 0
        self.rangeFront = 0
        self.rangeBack = 0
        self.intensity = 0
        self.last_intensity = 0

        # Create subscribers
        rospy.Subscriber('bcf_intensity', LightData, callback=self.intensity_callback)
        rospy.Subscriber('bcf_state', KalmanPositionEst, callback=self.state_callback)
        rospy.Subscriber('bcf_range', RangeData, callback=self.range_callback)
        
        # rospy.spin()
    
    def intensity_callback(self, data):
        self.intensity = data.intensity
        self.d_intensity.append(self.intensity)
        self.last_intensity = sum(self.d_intensity)/len(self.d_intensity)
        return

    def state_callback(self, data):
        self.x = data.stateX
        self.y = data.stateY
        self.z = data.stateZ
        return

    def range_callback(self, data):
        self.rangeLeft = data.left
        self.rangeRight = data.right
        self.rangeFront = data.front
        self.rangeBack = data.back
        return
    
    
    def run(self):
        return None

    def tumble(self):
        return None

if __name__ == '__main__':
    bcrazy = bcfController()
    time.sleep(3)
    while 1:
        time.sleep(0.25)
        print("[{},{},{}],l={},last={},[{},{},{},{}]".format(self.x,self.y,self.z,self.intensity,self.last_intensity,self.rangeLeft, self.rangeFront,self.rangeRight,self.rangeBack))
        print(len(self.d_intensity))
  

