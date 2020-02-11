import numpy as np
import pygame as pg
import pygame.locals as pglocs
from math import sin, cos, atan2, sqrt, pi
import random

# Screen
screen_width = 700
screen_height = 700
screen = pg.display.set_mode([screen_width, screen_height],
                             pglocs.DOUBLEBUF)

# Parameters for robot
vmax = 0.3  # maximum velocity of robot
K_p_ao = 1.8  # Kp for obstacle avoidance heading controller
K_p_t = 0.3  # Kp for tumble heading controller
skirt_r = 40  # Paranoid behavior theshold
ang_tol = 0.001
ao_scaling = 5e-5  # velocity scaling for obstacle avoidance

# Light source
src_pos = np.array([int(screen_width/2), int(screen_height/2)])  # position
sensor_dev = 0  # standard deviation for sensor uncertainity
intensity_scaling = 1e5  # scaling for inverse square law
init_intensity = 1e-4

# Circular obstacles
num_obsts = 5
obsts_min_radius = 20
obsts_max_radius = 60
obsts = []
radius = []
obsts_x = []
obsts_y = []
for i in range(num_obsts):
    radius.append(random.randint(obsts_min_radius, obsts_max_radius))
    obsts_x.append(random.randint(radius[i], screen_width - radius[i]))
    obsts_y.append(random.randint(radius[i], screen_height - radius[i]))


class robot():
    def __init__(self, init_pos, init_ang, size, intensity=init_intensity):
        '''
        Arguments: [init_pos_x, init_pos_y], init_angle, [l, b], intensity
        '''
        self.x = init_pos[0]
        self.y = init_pos[1]
        self.pos = [self.x, self.y]
        self.phi = init_ang
        self.intensity = intensity
        self.length = size[0]
        self.breadth = size[1]

        self.tip = [int(self.x + self.length * cos(self.phi)),
                    int(self.y + self.length * sin(self.phi))]
        self.bottom = [int(self.x - self.length * cos(self.phi)),
                       int(self.y - self.length * sin(self.phi))]
        self.bottom_l = [int(self.bottom[0] - self.breadth * sin(self.phi)),
                         int(self.bottom[1] + self.breadth * cos(self.phi))]
        self.bottom_r = [int(self.bottom[0] + self.breadth * sin(self.phi)),
                         int(self.bottom[1] - self.breadth * cos(self.phi))]

    def show(self):
        pg.draw.polygon(screen, (255, 0, 0),
                        [self.tip, self.bottom_l, self.bottom_r], 0)

    def run(self):
        v = vmax
        omega = 0
        return [v, omega]

    def tumble(self):
        v = 0
        phi_d = self.phi + (2*pi)*random.random()
        # phi_d = self.phi + pi/2
        omega = K_p_t*atan2(sin(phi_d - self.phi),
                            cos(phi_d - self.phi))  # P controller for omega
        if(abs(self.phi - phi_d < ang_tol)):
            v = vmax
        return [v, omega]

    def avoid_obst(self, obst_pos):
        e = obst_pos - self.pos  # error in position
        K = vmax * (1 - np.exp(- ao_scaling * np.linalg.norm(e)**2))\
            / np.linalg.norm(e)  # Scaling for velocity
        v = np.linalg.norm(K * e)  # Vel decreases as bot gets closer to obst
        phi_d = -atan2(e[1], e[0])  # Desired heading
        omega = K_p_ao*atan2(sin(phi_d - self.phi),
                             cos(phi_d - self.phi))  # P controller for omega
        return [v, omega]


class obstacle():
    def __init__(self, radius, pos):
        '''
        Arguments: radius, [pos_x, pos_y]
        '''
        self.r = radius
        self.x = pos[0]
        self.y = pos[1]

    def show(self):
        pg.draw.circle(screen, (0, 0, 255), (self.x, self.y), self.r, 0)


def simulate_light_sensor(robot_pos, src_pos, std_dev):
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]
    src_x = src_pos[0]
    src_y = src_pos[1]
    dist = sqrt((robot_x - src_x)**2 + (robot_y - src_y)**2)
    return intensity_scaling/(dist**2) + random.gauss(0, std_dev)


def main():
    # PyGame inits
    pg.init()
    pg.display.set_caption('Run and tumble simulation')
    # clock = pg.time.Clock()
    # ticks = pg.time.get_ticks()
    # frames = 0

    robot_x = 100  # Initial position
    robot_y = 600  # Initial position
    robot_phi = 0  # Initial angle
    robot_l = 15  # Robot length
    robot_b = 6  # Robot width
    intensity_last = 0

    # PyGame loop
    while(1):
        # To exit
        event = pg.event.poll()
        if(event.type == pglocs.QUIT or
           (event.type == pglocs.KEYDOWN and event.key == pglocs.K_ESCAPE)):
            break
        screen.fill((50, 55, 60))  # background

        # Draw robot, sensor skirt, obstacles and light source
        bot = robot([robot_x, robot_y], robot_phi,
                    [robot_l, robot_b])
        intensity = simulate_light_sensor([bot.x, bot.y], src_pos, sensor_dev)
        pg.draw.circle(screen, (100, 100, 100),
                       (int(bot.x), int(bot.y)), skirt_r, 0)  # sensor skirt
        for i in range(num_obsts):
            obsts.append(obstacle(radius[i], [obsts_x[i], obsts_y[i]]))
            obsts[i].show()
        bot.show()
        pg.draw.circle(screen, (0, 255, 0), src_pos, 8, 0)  # Draw light source
        # Constant intensity circles
        pg.draw.circle(screen, (250, 250, 250), src_pos, 20, 1)
        pg.draw.circle(screen, (200, 200, 200), src_pos, 50, 1)
        pg.draw.circle(screen, (150, 150, 150), src_pos, 100, 1)
        pg.draw.circle(screen, (100, 100, 100), src_pos, 150, 1)
        pg.draw.circle(screen, (60, 60, 60), src_pos, 200, 1)

        # Check if obstacles are in sensor skirt
        close_obst = []  # list of obstacles in sensor skirt [x, y, r]
        dist = []  # distances to those obstacles
        for i in range(num_obsts):
            d = sqrt((obsts[i].x - robot_x)**2 + (obsts[i].y - robot_y)**2)
            if(d <= (skirt_r + obsts[i].r)):
                close_obst.append([obsts[i].x, obsts[i].y, obsts[i].r])
                dist.append(d)

        # Run and tumble
        if(len(close_obst) == 0):  # No obstacle in sensor skirt
            if(intensity > intensity_last):
                [v, omega] = bot.run()  # controller run()
            else:
                [v, omega] = bot.tumble()  # controller tumble()
        # Paranoid behavior - run away from obstacle
        else:
            closest_obj = dist.index(min(dist))  # index of the closest object
            obst_pos = np.array([obsts_x[closest_obj], obsts_y[closest_obj]])
            [v, omega] = bot.avoid_obst(obst_pos)

        # Update robot pose as per control input and intensity_last
        robot_x += v*cos(bot.phi)
        robot_y += v*sin(bot.phi)
        robot_phi += omega
        intensity_last = intensity

        # FPS. Print if required
        # clock.tick(300)     # To limit fps, controls speed of the animation
        # fps = (frames*1000)/(pygame.time.get_ticks() - ticks)   # calculate current fps

        # Update PyGame display
        pg.display.flip()
        # frames+=1


if(__name__ == '__main__'):
    main()
