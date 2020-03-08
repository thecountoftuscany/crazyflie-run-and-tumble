import rospy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from rospy_crazyflie.msg import KalmanPositionEst

class livePlotter():
    def __init__(self):

        # Attributes
        self.plotArray = np.array([0,0,0])

        # Initialize variables to be published with message type
        self.state = KalmanPositionEst()
        self.intensity = LightData()
        self.action = ActionPub()

        # Initialize subscriber
        rospy.init_node('live_plotter')
        rospy.Subscriber('bcf_state', KalmanPositionEst, self.state_callback)
        rospy.Subscriber('bcf_action', ActionPub, self.action_callback)
        rospy.Subscriber('bcf_intensity', LightData, self.intensity_callback)
        
        # Set up the figure, the axis, and the plot element we want to animate
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')

        # Animate
        anim_object = animation.FuncAnimation(self.fig, self.animate, interval=25, save_count=200)
        plt.show()
        #anim_object.save('cf_animation.mp4', fps=10, extra_args=['-vcodec', 'libx264'])

        # Continue until node killed
        rospy.spin()

    def intensity_callback(self, data):
        #print("{},{},{}".format(data.stateX, data.stateY, data.stateZ))
        asdf
        self.pose = np.vstack(([self.pose,np.array([data.stateX,data.stateY,data.stateZ])]))

    def animate(self, i):
        self.ax.clear()
        self.ax.plot(self.pose[1:,0],self.pose[1:,1],self.pose[1:,2])

if __name__=='__main__':
    plotter=livePlotter()
