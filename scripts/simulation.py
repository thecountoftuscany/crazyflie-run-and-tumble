# This part gets rid of the 'Hello from PyGame...' message
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import numpy as np
import argparse
import pygame as pg
import pygame.locals as pglocs
from math import sin, cos, tan, atan2, sqrt, pi
import random
import time
from collections import deque

parser = argparse.ArgumentParser(description='Script for simulating run-and-tumble source seeking + obstacle avoidance algorithm. Press space to pause')
parser.add_argument('-v', '--velocity', type=float, default=0.3, help='Linear velocity of the simulated robot (pixels per pygame loop)')
parser.add_argument('-a', '--angular_v', type=float, default=0.005, help='Angular velocity of the simulated robot (radians per pygame loop)')
parser.add_argument('-d', '--ao_threshold', type=int, default=40, help='Threshold distance to trigger obstacle avoidance (in pixels)')
parser.add_argument('-rt', '--run_time', type=float, default=0.010, help='Amount of time to run before checking state again (in sec)')
parser.add_argument('-aot', '--ao_time', type=float, default=0.100, help='Amount of time to move away from obstacle before checking state again (in sec)')
parser.add_argument('-std', '--sensor_std', type=float, default=0, help='Standard deviation for sensor uncertainity')
parser.add_argument('-is', '--intensity_scaling', type=float, default=1e5, help='Scaling for inverse square law')
parser.add_argument('-sig_t', '--src_dec_thresh', type=float, default=1e4, help='The value of signal potential at which source is declared as reached')
parser.add_argument('-o', '--num_obsts', type=int, default=5, help='The number of obstacles to place')
parser.add_argument('-ominr', '--min_r', type=int, default=20, help='The minimum radius of obstacles (in pixels)')
parser.add_argument('-omaxr', '--max_r', type=int, default=60, help='The maximum radius of obstacles (in pixels)')
parser.add_argument('-l', '--light', action='store_true', default=False, help='Enable light background. Useful for publication. Default is dark for screens')
parser.add_argument('-rs', '--random_seed', type=int, help='Execute with a given seed for random (to reproduce and inspect the behaviour with the same configuration)')
args = parser.parse_args()

if args.random_seed is not None:
    random.seed(args.random_seed)

# Screen
screen_width = 700
screen_height = 700
screen = pg.display.set_mode([screen_width, screen_height],
                             pglocs.DOUBLEBUF)

# Signal source at the center
src_pos = np.array([int(screen_width/2), int(screen_height/2)])  # Position

def constrain(angle):
    '''
    Constrains given angle to [0, 2pi]
    '''
    while (angle < 0):
        angle += 2*pi
    while (angle >= 2*pi):
        angle -= 2*pi
    return angle


def simulate_field_sensor(robot_pos, src_pos, std_dev):
    '''
    Simulates a field sensor. The field could be light, temperature or whatever source is being seeked
    Inputs - [robot.x, robot.y], [src.x, src.y], std_deviation
    Returns - Field intensity (inverse square to distance, with noise)
    '''
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]
    src_x = src_pos[0]
    src_y = src_pos[1]
    dist = sqrt((robot_x - src_x)**2 + (robot_y - src_y)**2)
    return args.intensity_scaling/(dist**2) + random.gauss(0, std_dev)


def dist(pt1, pt2):
    '''
    Finds distance between two points
    '''
    return np.linalg.norm(pt1-pt2)


def simulate_rangefinder(robot, obsts):
    '''
    Simulates rangefinder.
    Inputs - robot object, list of all obstacle objects
    Returns - [left_dist, front_dist, right_dist, back_dist]
    '''
    xr = robot.x
    yr = robot.y
    phi_r = robot.phi

    dist = []  # Left, front, right, back
    angles = [phi_r - pi/2, phi_r,
              phi_r + pi/2, phi_r + pi]  # Angles of the 4 sensors
    for phi in angles:
        # Constrain angles within [0, 2pi]
        phi = constrain(phi)

        obsts_dists = []  # Will hold distances to all obstacles hit by beam
        for i in range(len(obsts)):
            xo = obsts[i].x
            yo = obsts[i].y
            r = obsts[i].r
            # Intersection of beam with obstacle
            # Solve beam eqn with obstacle circle eqn
            # Quadratic in x coordinate. Coeffs for ax^2+bx+c=0:
            a = 1 + (tan(phi))**2
            b = 2*(tan(phi)*(yr-xr*tan(phi)) - xo - yo*tan(phi))
            c = (yr-xr*tan(phi))*(yr-xr*tan(phi)-2*yo) + xo**2 + yo**2 - r**2
            if(b**2 - 4*a*c == 0):  # Beam is tangent to obstacle
                # Coords of point where beam hits obstacle
                xhit = -b/(2*a)
                yhit = xhit*tan(phi) + yr - xr*tan(phi)  # From eqn of beam
                # Check if obstacle is in front of the sensor
                dot_prod = cos(phi)*(xo-xr) + sin(phi)*(yo-yr)
                if(dot_prod > 0):
                    obsts_dists.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
            elif(b**2 - 4*a*c > 0):  # Beam hits two points on obstacle
                # Coords of points where beam hits obstacle
                xhit1 = (-b + sqrt(b**2-4*a*c))/(2*a)
                xhit2 = (-b - sqrt(b**2-4*a*c))/(2*a)
                yhit1 = xhit1*tan(phi) + yr - xr*tan(phi)  # From eqn of beam
                yhit2 = xhit2*tan(phi) + yr - xr*tan(phi)  # From eqn of beam
                dist1 = sqrt((xr-xhit1)**2 + (yr-yhit1)**2)
                dist2 = sqrt((xr-xhit2)**2 + (yr-yhit2)**2)
                # Check if obstacle is in front of the sensor
                dot_prod = cos(phi)*(xo-xr) + sin(phi)*(yo-yr)
                if(dot_prod > 0):
                    obsts_dists.append(min(dist1, dist2))  # Closer point
        if(len(obsts_dists) > 0):  # Beam hits one or more obstacles
            dist.append(min(obsts_dists))  # Choose closest obstacle
        else:  # Beam hits room walls
            if(phi < pi/2 or phi > 3*pi/2):  # Beam facing right wall
                # Coords of point where beam hits right edge
                xhit = screen_width
                yhit = xhit*tan(phi) + yr - xr*tan(phi)  # From eqn of beam
                if(yhit >= 0 and yhit <= screen_height):  # Hits r-wall in room
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
                else:  # Beam hits top or bottom wall
                    if(yhit > screen_height):  # Beam hits bottom wall
                        # Coords of point where beam hits bottom wall
                        yhit = screen_height
                        xhit = xr + (yhit-yr)/(tan(phi))  # From eqn of beam
                    else:  # Beam hits top wall
                        # Coords of point where beam hits top wall
                        yhit = 0
                        xhit = xr - yr/tan(phi)  # From equation of beam
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
            elif(phi > pi/2 and phi < 3*pi/2):  # Beam facing left edge
                # Coords of point where beam hits left edge
                xhit = 0
                yhit = yr - xr*tan(phi)  # From equation of beam
                if(yhit >= 0 and yhit <= screen_width):  # Hits l-wall in room
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
                else:  # Beam hits top or bottom wall
                    if(yhit > screen_height):  # Beam hits bottom wall
                        # Coords of point where beam hits bottom wall
                        yhit = screen_height
                        xhit = xr + (yhit-yr)/(tan(phi))  # From eqn of beam
                    else:  # Beam hits top wall
                        yhit = 0
                        xhit = xr - yr/tan(phi)  # From equation of beam
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
            elif(phi == pi/2):  # Beam directly towards bottom wall
                dist.append(screen_height - yr)
            elif(phi == 3*pi/2):  # Beam directly facing towards top wall
                dist.append(yr)

    return dist


def pgloop(inputs, t=args.run_time):
    '''
    PyGame loop
    '''
    v = inputs[0]
    dir_x = inputs[1]
    dir_y = inputs[2]
    omega = inputs[3]
    init_time = time.time()
    while(time.time() < init_time + t):  # Do this for t seconds
        event = pg.event.poll()
        # Pause if SPC is pressed
        if(event.type == pglocs.KEYDOWN and event.key == pglocs.K_SPACE):
            while(1):
                event = pg.event.poll()
                # Resume if SPC pressed again
                if(event.type == pglocs.KEYDOWN and
                     event.key == pglocs.K_SPACE):
                    break  
                time.sleep(0.010)  # Wait for 10 ms
        if args.light:
            screen.fill((255, 255, 255))  # white background
        else:
            screen.fill((50, 55, 60))  # dark background

        draw()

        # Update robot attributes
        bot.x += v*dir_x
        bot.y += v*dir_y
        bot.phi = constrain(bot.phi + omega)
        bot.tip = [int(bot.x + bot.length * cos(bot.phi)),
                   int(bot.y + bot.length * sin(bot.phi))]
        bot.bottom = [int(bot.x - bot.length * cos(bot.phi)),
                      int(bot.y - bot.length * sin(bot.phi))]
        bot.bottom_l = [int(bot.bottom[0] - bot.breadth * sin(bot.phi)),
                        int(bot.bottom[1] + bot.breadth * cos(bot.phi))]
        bot.bottom_r = [int(bot.bottom[0] + bot.breadth * sin(bot.phi)),
                        int(bot.bottom[1] - bot.breadth * cos(bot.phi))]
        bot.intensity_last = bot.intensity

        # Update multiranger attributes
        distances = simulate_rangefinder(bot, obsts)
        mr.x = bot.x
        mr.y = bot.y
        mr.phi = bot.phi
        mr.ld = distances[0]
        mr.fd = distances[1]
        mr.rd = distances[2]
        mr.bd = distances[3]
        mr.lpoint = np.array([mr.x+mr.ld*cos(mr.phi - pi/2),
                              mr.y+mr.ld*sin(mr.phi - pi/2)])
        mr.fpoint = np.array([mr.x+mr.fd*cos(mr.phi),
                              mr.y+mr.fd*sin(mr.phi)])
        mr.rpoint = np.array([mr.x+mr.rd*cos(mr.phi + pi/2),
                              mr.y+mr.rd*sin(mr.phi + pi/2)])
        mr.bpoint = np.array([mr.x+mr.bd*cos(mr.phi + pi),
                              mr.y+mr.bd*sin(mr.phi + pi)])

        # Update obstacle attributes
        # for i in range(args.num_obsts):
        #     obsts[i].x += int(1.5 * sin(0.02*pg.time.get_ticks()))
        #     obsts[i].y += int(1.5 * sin(0.02*pg.time.get_ticks()))

        # FPS. Print if required
        # clock.tick(300)     # To limit fps, controls speed of the animation
        # fps = (frames*1000)/(pg.time.get_ticks() - ticks)  # calculate fps

        # Update PyGame display
        pg.display.flip()
        # frames+=1


class robot():
    def __init__(self, init_pos, init_ang, size):
        '''
        Args: [init_x, init_y], init_ang, [l, b]
        '''
        self.x = init_pos[0]
        self.y = init_pos[1]
        self.pos = np.array([self.x, self.y])
        self.phi = init_ang
        self.intensity = random.random()
        self.intensity_last = 0
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
        '''
        Draw robot to the screen
        '''
        # Robot
        # pg.draw.polygon(screen, (255, 0, 0),
                        # [self.tip, self.bottom_l, self.bottom_r], 0)  # red
        pg.draw.polygon(screen, (200, 0, 0),
                        [self.tip, self.bottom_l, self.bottom_r], 0)  # soft red
        # Center pt
        pg.draw.circle(screen, (250, 180, 0), [int(self.x), int(self.y)], 1)

    def run(self):
        '''
        Run controller
        '''
        pgloop(bot.start_forward())

    def tumble(self, ang=1.5):
        '''
        Tumble controller
        '''
        tumble_time = ang*random.random()
        if(random.random() <= 0.5):
            pgloop(bot.start_turn_right(), tumble_time)
        else:
            pgloop(bot.start_turn_left(), tumble_time)
        pgloop(bot.start_forward())
        return tumble_time

    def avoid_obst(self, mrindex):
        '''
        Avoid obstacle controller
        '''
        if(mrindex == 0):  # Smallest dist sensed from left
            pgloop(bot.start_right(), args.ao_time)
            pgloop(bot.start_turn_right())
            pgloop(bot.start_forward())
        elif(mrindex == 1):  # Smallest dist sensed from front
            pgloop(bot.start_back(), args.ao_time)
            pgloop(bot.start_turn_right())
            pgloop(bot.start_forward())
        elif(mrindex == 2):  # Smallest dist sensed from right
            pgloop(bot.start_left(), args.ao_time)
            pgloop(bot.start_turn_left())
            pgloop(bot.start_forward())
        else:  # Smallest dist sensed from back
            pgloop(bot.start_forward())

    ########################################
    # Crazyflie client methods
    ########################################
    # Unimplemented take_off(), land(), wait()

    # Velocity control commands
    ####################
    # Unimplemented:
    # start_up(), start_down()
    # start_circle_left(), start_circle_right()

    def start_left(self, velocity=args.velocity):
        '''
        Start moving left
        '''
        theta = constrain(self.phi - pi/2)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_right(self, velocity=args.velocity):
        '''
        Start moving right
        '''
        theta = constrain(self.phi + pi/2)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_forward(self, velocity=args.velocity):
        '''

        Start moving forward
        '''
        theta = constrain(self.phi)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_back(self, velocity=args.velocity):
        '''

        Start moving backward
        '''
        theta = constrain(self.phi - pi)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def stop(self):
        '''
        STOP!
        '''
        velocity = 0
        dir_x = 0
        dir_y = 0
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_turn_left(self, rate=args.angular_v):
        '''
        Start turning left
        '''
        velocity = 0
        dir_x = 0
        dir_y = 0
        omega = -rate
        return [velocity, dir_x, dir_y, omega]

    def start_turn_right(self, rate=args.angular_v):
        '''
        Start turning right
        '''
        velocity = 0
        dir_x = 0
        dir_y = 0
        omega = rate
        return [velocity, dir_x, dir_y, omega]

    def start_linear_motion(self, vel_x, vel_y):
        '''
        Start a linear motion
        Positive X is forward
        Positive Y is left
        '''
        velocity = sqrt(vel_x**2 + vel_y**2)
        theta = constrain(self.phi - atan2(vel_y, vel_x))
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]


class obstacle():
    def __init__(self, radius, pos):
        '''
        Args: radius, [pos_x, pos_y]
        '''
        self.r = radius
        self.x = pos[0]
        self.y = pos[1]

    def show(self):
        # pg.draw.circle(screen, (0, 0, 255), (self.x, self.y), self.r, 0)  # blue
        pg.draw.circle(screen, (0, 0, 200), (self.x, self.y), self.r, 0)  # soft blue


class multiranger():
    def __init__(self, robot, dists):
        '''
        Args: robot object, [left_dist, front_dist, right_dist, back_dist]
        '''
        self.x = robot.x
        self.y = robot.y
        self.phi = robot.phi
        self.ld = dists[0]
        self.fd = dists[1]
        self.rd = dists[2]
        self.bd = dists[3]
        self.lpoint = np.array([self.x+self.ld*cos(self.phi - pi/2),
                                self.y+self.ld*sin(self.phi - pi/2)])
        self.fpoint = np.array([self.x+self.fd*cos(self.phi),
                                self.y+self.fd*sin(self.phi)])
        self.rpoint = np.array([self.x+self.rd*cos(self.phi + pi/2),
                                self.y+self.rd*sin(self.phi + pi/2)])
        self.bpoint = np.array([self.x+self.bd*cos(self.phi + pi),
                                self.y+self.bd*sin(self.phi + pi)])

    def show(self):
        '''
        Draw multiranger beams to the screen
        '''
        # Left sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x), int(self.y)],
                     [int(self.lpoint[0]), int(self.lpoint[1])], 2)
        # Front sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x), int(self.y)],
                     [int(self.fpoint[0]), int(self.fpoint[1])], 2)
        # Right sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x), int(self.y)],
                     [int(self.rpoint[0]), int(self.rpoint[1])], 2)
        # Back sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x), int(self.y)],
                     [int(self.bpoint[0]), int(self.bpoint[1])], 2)


# Create obstacles
obsts = []
radius = []
obsts_x = []
obsts_y = []
for i in range(args.num_obsts):
    radius.append(random.randint(args.min_r, args.max_r))
    obsts_x.append(random.randint(radius[i], screen_width - radius[i]))
    obsts_y.append(random.randint(radius[i], screen_height - radius[i]))
for i in range(args.num_obsts):
    obsts.append(obstacle(radius[i], [obsts_x[i], obsts_y[i]]))

# Could be added to argparse, but don't really see the point
robot_x = 0.9 * screen_width * random.random()  # Initial position
robot_y = 0.9 * screen_height * random.random()  # Initial position
robot_phi = 2 * pi * random.random()  # Initial angle
robot_l = 15  # Robot length
robot_b = 6  # Robot width

# Initialize objects
bot = robot([robot_x, robot_y], robot_phi,
            [robot_l, robot_b])
distances = simulate_rangefinder(bot, obsts)
mr = multiranger(bot, distances)

def draw():
    '''
    Draw stuff to PyGame screen
    '''
    # Threshold radius
    # pg.draw.circle(screen, (100, 100, 100),
                   # (int(bot.x), int(bot.y)),
                   # args.ao_threshold, 0)
    pg.draw.circle(screen, (200, 200, 200),
                   (int(bot.x), int(bot.y)),
                   args.ao_threshold, 0)  # softer grey for white background
    # Constant intensity circles
    # pg.draw.circle(screen, (250, 250, 250), src_pos, 20, 1)
    # pg.draw.circle(screen, (200, 200, 200), src_pos, 50, 1)
    # pg.draw.circle(screen, (150, 150, 150), src_pos, 100, 1)
    # pg.draw.circle(screen, (100, 100, 100), src_pos, 150, 1)
    # pg.draw.circle(screen, (60, 60, 60), src_pos, 200, 1)
    # pg.draw.circle(screen, (40, 40, 40), src_pos, 250, 1)
    # Multiranger beams
    mr.show()
    # Robot
    bot.show()
    # Obstacles
    for i in range(len(obsts)):
        obsts[i].show()
    # Signal source
    pg.draw.circle(screen, (0, 200, 0), src_pos, 8, 0)  # soft green


def main():
    '''
    The main function
    '''
    # PyGame inits
    pg.init()
    pg.display.set_caption('Run and tumble simulation')
    # clock = pg.time.Clock()
    # ticks = pg.time.get_ticks()
    # frames = 0

    # For plotting
    t = 0
    rollout_t = np.array([t])
    rollout_pos = np.array([[robot_x, robot_y]])
    rollout_dist = np.array([dist(bot.pos, src_pos)])
    rollout_action = np.array([1])

    # Commands
    while(bot.intensity < args.src_dec_thresh):
        # Read sensor measurement
        bot.intensity = simulate_field_sensor([bot.x, bot.y],
                                              src_pos, args.sensor_std)

        # The Finite State Machine #
        # If no obsts in args.ao_threshold, run-and-tumble
        if(mr.ld > args.ao_threshold and mr.fd > args.ao_threshold and
           mr.rd > args.ao_threshold and mr.bd > args.ao_threshold):
            # If intensity is increasing, run
            if(bot.intensity > bot.intensity_last):
                dt = args.run_time
                bot.run()
                action = 1
            # Else tumble to random direction
            else:
                dt = bot.tumble()
                action = 2
        # Obst(s) detected within args.ao_threshold. Run away from closest obst
        else:
            dt = args.ao_time
            # Closest dist sensed among the 4 directions
            mrmin = min(mr.ld, mr.fd, mr.rd, mr.bd)
            mrindex = [mr.ld, mr.fd, mr.rd, mr.bd].index(mrmin)
            bot.avoid_obst(mrindex)
            action = 3
        # Update rollout_ variables and save to file for plotting
        t += dt
        rollout_t = np.append(rollout_t, t)
        rollout_pos = np.vstack((rollout_pos, np.array([bot.x, bot.y])))
        rollout_dist = np.append(rollout_dist, dist(src_pos, np.array([bot.x, bot.y])))
        rollout_action = np.append(rollout_action, action)
    # To save to file for plotting
    obstacles = np.zeros((1, 3))  # vstack of [x, y, r]
    for obst in obsts:
        obstacles = np.vstack((obstacles, np.array([obst.x, obst.y, obst.r])))
    obstacles = obstacles[1:]
    # np.savez('../data/rollout_s_' + str(seed) + '_' +
             # str(time.strftime('%Y%m%d_%H-%M-%S')) +
             # '.npz', rollout_t=rollout_t, rollout_pos=rollout_pos,
             # rollout_dist=rollout_dist, rollout_action=rollout_action,
             # obstacles=obstacles)

    # If intensity >= args.src_dec_thresh, stop
    pgloop(bot.stop())

    # Congratulatory screen
    time.sleep(3)
    if args.light:
        screen.fill((255, 255, 255))  # white background
    else:
        screen.fill((50, 55, 60))  # dark background
    font = pg.font.SysFont("Hack", 72)
    success_text = font.render("SUCCESS!!!", True, (0, 128, 0))
    screen.blit(success_text,
                ((screen_width - success_text.get_width())//2,
                    (screen_height - success_text.get_height())//2))
    pg.display.flip()
    time.sleep(3)


if(__name__ == '__main__'):
    main()
