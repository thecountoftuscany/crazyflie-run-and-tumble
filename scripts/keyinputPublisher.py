import pygame
import sys
import rospy
from std_msgs.msg import String
from pygame.locals import *
from actionlib_msgs.msg import GoalStatusArray

class cfPublisher():
    def __init__(self):
        # Set up node, pub, subs
        self.status = GoalStatusArray()
        rospy.init_node('keyinputpublisher')
        rospy.Subscriber('crazyflie1/position_control/status',GoalStatusArray, self.status_callback)
        self.inputpublisher = rospy.Publisher('keyinput',String,queue_size=2)
        self.rate = rospy.Rate(500)
        pygame.init()
        self.win_width=320
        self.win_height=240
        self.screen = pygame.display.set_mode((self.win_width, self.win_height), DOUBLEBUF)
        pygame.display.set_caption("Crazyflie control")
        #frames = 0
        #ticks = pygame.time.get_ticks()
        self.screen.fill((50, 55, 60))       # background
        self.titlefont = pygame.font.SysFont('hack', 20)
        self.text = self.titlefont.render('Keyboard input logger for controlling crazyflie', True, (255, 255, 255)); self.screen.blit(self.text, (8,10))
        self.descfont = pygame.font.SysFont('hack', 18)
        self.text = self.descfont.render('Keys: w,s,a,d> move;   q,e> turn;   f> stop;   z>land', True, (255, 255, 255)); self.screen.blit(self.text, (5, self.win_height/2 - 10))
        #self.text = self.descfont.render('Press Esc to quit.', True, (255, 255, 255)); self.screen.blit(self.text, (5,self.win_height/2 + 10))
        self.kinput = String()
        #pygame.key.set_repeat(20)
        pygame.display.flip()
        rospy.spin()
        #print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks() - ticks)))

    def status_callback(self,data):
        try:
            self.status = data.status_list[-1].status
        except:
            self.status = 3
        # print(self.status)

        self.event = pygame.event.poll()
        # if self.event.type == QUIT or (self.event.type == KEYDOWN and self.event.key == K_ESCAPE):
            # self.kinput = 'z'
            # self.inputpublisher.publish(self.kinput)
            # sys.exit(0)
        if self.event.type==KEYDOWN and self.event.key==K_w:
            self.kinput = 'w'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_s:
            self.kinput = 's'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_a:
            self.kinput = 'a'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_d:
            self.kinput = 'd'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_q:
            self.kinput = 'q'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_e:
            self.kinput = 'e'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_f:
            self.kinput = 'f'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        elif self.event.type==KEYDOWN and self.event.key==K_z:
            self.kinput = 'z'
            if(self.status==3):
                self.inputpublisher.publish(self.kinput)
        # else:
            # kinput=''
            # inputpublisher.publish(kinput)
        #rate.sleep()
        #pygame.display.flip()
        #frames += 1

if __name__=='__main__':
    publisher=cfPublisher()
