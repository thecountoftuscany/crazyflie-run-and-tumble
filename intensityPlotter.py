import rospy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from rospy_crazyflie.msg import *

class livePlotter():
    def __init__(self):

        # Attributes
        self.plotArray = np.array([0,0,0])

        # Initialize variables to be published with message type
        self.state = KalmanPositionEst()
        self.intensity = LightData()
        self.action = ActionPub()
	self.x = 0
	self.y = 0
	self.z = 0

        # Initialize subscriber
        rospy.init_node('live_plotter')
        rospy.Subscriber('bcf_state', KalmanPositionEst, self.state_callback)
        rospy.Subscriber('bcf_action', ActionPub, self.action_callback)
        rospy.Subscriber('bcf_intensity', LightData, self.intensity_callback)
        
        # Set up the figure, the axis, and the plot element we want to animate
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')

        # Animate
        anim_object = animation.FuncAnimation(self.fig, self.animate, interval=125, save_count=200)
        plt.show()
        #anim_object.save('cf_animation.mp4', fps=10, extra_args=['-vcodec', 'libx264'])

        # Continue until node killed
        rospy.spin()

    def state_callback(self, data):
        self.x = data.stateX
        self.y = data.stateY
        self.z = data.stateZ
        #print("state = {},{},{}".format(self.x, self.y, self.z))

    def action_callback(self, data):
        self.action = data.action
        #print("action = {}".format(self.action))

    def intensity_callback(self, data):
        self.intensity = data.intensity
        #print("intensity = {}".format(self.intensity))
        self.plotArray = np.vstack(([self.plotArray, np.array([self.x, self.y, self.intensity])]))

    def animate(self, i):
        self.ax.clear()
	#self.ax.plot(self.plotArray[1:,0], self.plotArray[1:,1], self.plotArray[1:,2])
        if(self.action == 1):
            self.ax.plot(self.plotArray[1:,0], self.plotArray[1:,1], self.plotArray[1:,2], color=(0.0,1.0,0.0))
            pass
        elif(self.action == 2):
            self.ax.plot(self.plotArray[1:,0], self.plotArray[1:,1], self.plotArray[1:,2], color=(0.0,0.0,1.0))
            pass
        elif(self.action == 3):
            self.ax.plot(self.plotArray[1:,0], self.plotArray[1:,1], self.plotArray[1:,2], color=(1.0,0.0,0.0))
            pass

if __name__=='__main__':
    plotter=livePlotter()
