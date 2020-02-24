import rospy
import rospy_crazyflie.crazyflie_client as crazyflie_client
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.msg import KalmanPositionEst
import sys

class bCrazyflie:
    def __init__(self):
        # Init node
        rospy.init_node('bcf_sensor_publisher')

        # Attributes
        self.logvars = [LogVariable('stateEstimate.x', 'float'),
                        LogVariable('stateEstimate.y', 'float'),
                        LogVariable('stateEstimate.z', 'float'),
                        LogVariable('BH1750.intensity', 'float'),
                        LogVariable('range.left', 'float'),
                        LogVariable('range.front', 'float'),
                        LogVariable('range.right', 'float'),
                        LogVariable('range.back', 'float'),
                        LogVariable('range.up', 'float')]
        # Intensity and range publishers to be added later
        self.state_publisher = rospy.Publisher('bcf_state', KalmanPositionEst, queue_size=10)
        # self.intensity_publisher = rospy.Publisher('bcf_intensity', TYPE???, queue_size=10)
        # self.range_publisher = rospy.Publisher('bcf_range', TYPE???, queue_size=10)
        # Get all crazyflies on the /crazyflie_server
        self.crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
        # Connect to the first crazyflie
        self.client = CrazyflieClient(self.crazyflies[0])
        self.state = KalmanPositionEst()
        # self.intensity = TYPE???
        # self.range = TYPE???

        # Additional stuff
        # Configures the crazyflie to log the data
        self.client.add_log_config('run-and-tumble-config', self.logvars, 10, callback=self.sensorUpdate)
        # self.client.take_off(0.2)
        # self.client.wait()
        # rospy.spin()

    def sensorUpdate(self, data, timestamp):
        # Called by the CrazyflieClient object when new data is available
        self.state.stateX = data['stateEstimate.x']
        self.state.stateY = data['stateEstimate.y']
        self.state.stateZ = data['stateEstimate.z']
        self.state.cfstamp = timestamp
        self.intensity = data['BH1750.intensity']
        # self.range.left = data['range.left']
        # self.range.front = data['range.front']
        # self.range.right = data['range.right']
        # self.range.back = data['range.back']
        # self.range.up = data['range.up']
        # print("x={}, y={}, z={}, light={}, left={}, front={}, right={}, back={}".format(self.x, self.y, self.z, self.intensity, self.rangeLeft, self.rangeFront, self.rangeRight, self.rangeBack))
        self.state_publisher.publish(self.state)
        #self.intensity_publisher.publish(self.intensity)
        #self.range_publisher.publish(self.range)

if __name__=='__main__':
    bcf = bCrazyflie()
