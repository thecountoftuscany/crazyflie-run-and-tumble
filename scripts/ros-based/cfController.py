import rospy
from std_msgs.msg import String
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
#from rospy_crazyflie.msg import KalmanPositionEst
import time
import rospy_crazyflie.crazyflie_client as crazyflie_client
from rospy_crazyflie.crazyflie_client import CrazyflieClient
import sys

class CrazyflieController:
    def __init__(self):
        # Init node, publisher, subscriber objects
        rospy.init_node('cf_controller')
        rospy.Subscriber('keyinput', String, self.control_callback)

        # Atrributes
        self.cf_state_publisher = rospy.Publisher('cf_state', KalmanPositionEst, queue_size=10)
        self.config_name = 'stateEstimateconfig'
        self.variables = [LogVariable('stateEstimate.x', 'float'), LogVariable('stateEstimate.y', 'float'), LogVariable('stateEstimate.z', 'float')]
        self.period_in_ms = 10.
        #Get all crazyflies on the /crazyflie_server
        self.crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
        #Connect to first crazyflie
        self.client = CrazyflieClient(self.crazyflies[0])
        self.cf_state = KalmanPositionEst()

        # Additional stuff
        #Configures the crazyflie to log the data
        # self.client.add_log_config(self.config_name, self.variables, self.period_in_ms, callback = self.state_callback)
        self.client.take_off(.3)     # Takeoff to .3 meters
        self.client.wait()
        rospy.spin()

    def control_callback(self, data):
        # print("{}".format(data.data))
        if(data.data=='w'):
            self.client.wait()
            self.client.forward(1)
        elif(data.data=='s'):
            self.client.wait()
            self.client.back(0.1)
        elif(data.data=='a'):
            self.client.wait()
            self.client.left(0.1)
            self.client.wait()
        elif(data.data=='d'):
            self.client.wait()
            self.client.right(0.1)
            self.client.wait()
        elif(data.data=='q'):
            self.client.wait()
            self.client.turn_left(20)
            self.client.wait()
        elif(data.data=='e'):
            self.client.wait()
            self.client.turn_right(20)
            self.client.wait()
        elif(data.data=='f'):
            self.client.wait()
            self.client.stop()
            self.client.wait()
        elif(data.data=='z'):
            self.client.wait()
            self.client.land()
            self.client.wait()
            del self.client
            sys.exit(0)
        # else:
            # self.client.wait()

    def state_callback(self, data, timestamp):
            # This is called by the CrazyflieClient object when new data is available
            self.cf_state.stateX = data['stateEstimate.x']
            self.cf_state.stateY = data['stateEstimate.y']
            self.cf_state.stateZ = data['stateEstimate.z']
            self.cf_state.cfstamp = timestamp
            #print("{},{},{}".format(data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
            self.cf_state_publisher.publish(self.cf_state)

if __name__=='__main__':
    controller = CrazyflieController()
