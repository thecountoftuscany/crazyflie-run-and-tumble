import rospy
import rospy_crazyflie.crazyflie_client as crazyflie_client
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
import time
from collections import deque
import sys
import random
import numpy as np


num_samples = 10
takeoff_height = 0.3    # m
forward_vel = 0.1       # m/s
strafe_vel = 0.1        # m/s
strafe_time = 2	        # sec
turn_deg = 20           # deg
max_intensity = 500   	# lux
obst_dist_thresh = 500  # mm
run_time = 1            # sec

class bcfController:
    def __init__(self):
        # Init node
        rospy.init_node('bcf_Controller')

        # Connect to the crazyflie
        self.crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
        self.client = crazyflie_client.CrazyflieClient(self.crazyflies[0])

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
    
    def start(self):
        self.client.take_off(takeoff_height)
        self.client.wait()
        rospy.sleep(6.)
        self.client.start_forward(forward_vel)
        self.client.wait()
        rospy.sleep(1.)
        print('Start routine complete!')
        return None    

    def run(self):
	if(self.intensity < 1000):
		pass
        	self.client.start_forward(forward_vel)
	else:
		pass
		self.client.start_forward(forward_vel)
        print('Running!')
        rospy.sleep(1.)
        return None

    def tumble(self):
        self.client.wait()
        c = random.random()
        print('Tumbling! c = ',c)
        if c <= 0.5:
            self.client.turn_left(180*random.random()) # turns CF in the range 0-180 deg
        else:
            self.client.turn_right(180*random.random())
        rospy.sleep(0.5)
	self.client.start_forward(forward_vel)
        return None

    def avoid_obst(self):
        dist_arr = np.array([self.rangeLeft, self.rangeFront, self.rangeRight, self.rangeBack])
        smallest_dist = np.argmin(dist_arr)

        if smallest_dist == 0:
            print('Obstacle to left, moving right')
            self.client.wait()          
            self.client.start_right(strafe_vel)
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_right(turn_deg)
            self.client.wait()
            self.client.start_forward(forward_vel)
        elif smallest_dist == 1:
            print('Obstacle to front, moving back')
            self.client.wait()
            self.client.start_back(strafe_vel)                        
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_left(turn_deg)
            self.client.wait()
            self.client.start_forward(forward_vel)
        elif smallest_dist == 2:
            print('Obstacle to right, moving left') 
            self.client.wait()
            self.client.start_left(strafe_vel)
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_left(turn_deg)
            self.client.wait()
            self.client.start_forward(forward_vel)
        elif smallest_dist == 3:
            print('Obstacle to back, moving forward')   
            self.client.wait()         
            self.client.start_forward(forward_vel)        
        rospy.sleep(0.5)
        return None
    
    def stop(self):
        self.client.stop() 
        self.client.wait()       
        self.client.land()
        self.client.wait()
        del self.client
        sys.exit(0)
        return None

if __name__ == '__main__':
    bcrazy = bcfController()
    
    # Takeoff and hold
    bcrazy.start()
    
    # Start high level control loop
    while bcrazy.intensity < max_intensity:
        if (bcrazy.rangeLeft  > obst_dist_thresh and
            bcrazy.rangeRight > obst_dist_thresh and
            bcrazy.rangeFront > obst_dist_thresh and
            bcrazy.rangeBack  > obst_dist_thresh):
            print(bcrazy.intensity, bcrazy.last_intensity)
            if bcrazy.intensity > bcrazy.last_intensity:
                bcrazy.run()
            else:
                bcrazy.tumble()
        else:
            bcrazy.avoid_obst()

    # end while loop
    bcrazy.stop()
    


    while 1:
        time.sleep(0.25)
        print("[{},{},{}],l={},last={},[{},{},{},{}]".format(bcrazy.x,bcrazy.y,bcrazy.z,bcrazy.intensity,bcrazy.last_intensity,bcrazy.rangeLeft, bcrazy.rangeFront,bcrazy.rangeRight,bcrazy.rangeBack))
        print(len(bcrazy.d_intensity))
  

