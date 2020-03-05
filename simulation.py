import numpy as np
import pygame as pg
import pygame.locals as pglocs
from math import sin, cos, tan, atan2, sqrt, pi
import random
import time

# Screen
screen_width = 700
screen_height = 700
screen = pg.display.set_mode([screen_width, screen_height],
                             pglocs.DOUBLEBUF)

# Parameters for robot
vmax = 0.3  # maximum velocity of robot
def_vel = 0.3  # default velocity for crazyflie
def_rate = 0.005  # default angular velocity for crazyflie
K_p_ao = 1.8  # Kp for obstacle avoidance heading controller
K_p_t = 0.3  # Kp for tumble heading controller
dist_thresh = 40  # Paranoid behavior theshold
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


def constrain(angle):
    '''
    Constrains given angle to [0, 2pi]
    '''
    while (angle < 0):
        angle += 2*pi
    while (angle >= 2*pi):
        angle -= 2*pi
    return angle


def simulate_light_sensor(robot_pos, src_pos, std_dev):
    '''
    Simulates light sensor.
    Inputs - [robot.x, robot.y], [src.x, src.y], std_deviation
    Returns - Light intensity (inverse square to distance, with noise)
    '''
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]
    src_x = src_pos[0]
    src_y = src_pos[1]
    dist = sqrt((robot_x - src_x)**2 + (robot_y - src_y)**2)
    return intensity_scaling/(dist**2) + random.gauss(0, std_dev)


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
        '''
        Draw robot to the screen
        '''
        # Robot
        pg.draw.polygon(screen, (255, 0, 0),
                        [self.tip, self.bottom_l, self.bottom_r], 0)
        # Center pt
        pg.draw.circle(screen, (250, 180, 0), [int(self.x), int(self.y)], 1)

    def run(self):
        '''
        Run controller
        '''
        v = vmax
        omega = 0
        return [v, omega]

    def tumble(self):
        '''
        Tumble controller
        '''
        v = 0
        phi_d = self.phi + (2*pi)*random.random()
        # phi_d = self.phi + pi/2
        omega = K_p_t*atan2(sin(phi_d - self.phi),
                            cos(phi_d - self.phi))  # P controller for omega
        if(abs(self.phi - phi_d < ang_tol)):
            v = vmax
        return [v, omega]

    def avoid_obst(self, obst_pos):
        '''
        Avoid obstacle controller
        '''
        e = obst_pos - self.pos  # error in position
        K = vmax * (1 - np.exp(- ao_scaling * np.linalg.norm(e)**2))\
            / np.linalg.norm(e)  # Scaling for velocity
        v = np.linalg.norm(K * e)  # Vel decreases as bot gets closer to obst
        phi_d = -atan2(e[1], e[0])  # Desired heading
        omega = K_p_ao*atan2(sin(phi_d - self.phi),
                             cos(phi_d - self.phi))  # P controller for omega
        return [v, omega]

    ########################################
    # Crazyflie client methods
    ########################################
    # Unimplemented take_off(), land(), wait()

    # Velocity commands
    ####################
    # Unimplemented:
    # start_up(), start_down()
    # start_circle_left(), start_circle_right()

    def start_left(self, velocity=def_vel):
        '''
        Start moving left
        '''
        theta = constrain(self.phi - pi/2)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_right(self, velocity=def_vel):
        '''
        Start moving right
        '''
        theta = constrain(self.phi + pi/2)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_forward(self, velocity=def_vel):
        '''

        Start moving forward
        '''
        theta = constrain(self.phi)
        dir_x = cos(theta)
        dir_y = sin(theta)
        omega = 0
        return [velocity, dir_x, dir_y, omega]

    def start_back(self, velocity=def_vel):
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

    def start_turn_left(self, rate=def_rate):
        '''
        Start turning left
        '''
        velocity = 0
        dir_x = 0
        dir_y = 0
        omega = -rate
        return [velocity, dir_x, dir_y, omega]

    def start_turn_right(self, rate=def_rate):
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
        Arguments: radius, [pos_x, pos_y]
        '''
        self.r = radius
        self.x = pos[0]
        self.y = pos[1]

    def show(self):
        pg.draw.circle(screen, (0, 0, 255), (self.x, self.y), self.r, 0)


class multiranger():
    def __init__(self, robot, dists):
        '''
        Arguments: robot object, [left_dist, front_dist, right_dist, back_dist]
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
for i in range(num_obsts):
    radius.append(random.randint(obsts_min_radius, obsts_max_radius))
    obsts_x.append(random.randint(radius[i], screen_width - radius[i]))
    obsts_y.append(random.randint(radius[i], screen_height - radius[i]))
for i in range(num_obsts):
    obsts.append(obstacle(radius[i], [obsts_x[i], obsts_y[i]]))


def main():
    # PyGame inits
    pg.init()
    pg.display.set_caption('Run and tumble simulation')
    # clock = pg.time.Clock()
    # ticks = pg.time.get_ticks()
    # frames = 0

    robot_x = 100  # Initial position
    robot_y = 600  # Initial position
    robot_phi = 5  # Initial angle
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
        # Pause if SPC is pressed
        if(event.type == pglocs.KEYDOWN and event.key == pglocs.K_SPACE):
            while(1):
                event = pg.event.poll()
                if(event.type == pglocs.KEYDOWN and
                   event.key == pglocs.K_SPACE):
                    break  # Resume if SPC pressed again
                time.sleep(0.010)  # Wait for 10 ms
        screen.fill((50, 55, 60))  # background

        # Draw robot, sensor skirt, obstacles and light source
        bot = robot([robot_x, robot_y], robot_phi,
                    [robot_l, robot_b])
        distances = simulate_rangefinder(bot, obsts)
        mr = multiranger(bot, distances)
        mr.show()
        intensity = simulate_light_sensor([bot.x, bot.y], src_pos, sensor_dev)
        pg.draw.circle(screen, (100, 100, 100),
                       (int(bot.x), int(bot.y)),
                       dist_thresh, 0)  # threshold radius
        for i in range(len(obsts)):
            obsts[i].show()
        bot.show()
        pg.draw.circle(screen, (0, 255, 0), src_pos, 8, 0)  # Draw light source

        # Constant intensity circles
        pg.draw.circle(screen, (250, 250, 250), src_pos, 20, 1)
        pg.draw.circle(screen, (200, 200, 200), src_pos, 50, 1)
        pg.draw.circle(screen, (150, 150, 150), src_pos, 100, 1)
        pg.draw.circle(screen, (100, 100, 100), src_pos, 150, 1)
        pg.draw.circle(screen, (60, 60, 60), src_pos, 200, 1)
        pg.draw.circle(screen, (40, 40, 40), src_pos, 250, 1)

        [v, dir_x, dir_y, omega] = bot.start_linear_motion(0, 0.3)
        # # Run and tumble
        # if(mr.ld > dist_thresh and mr.fd > dist_thresh and
        #    mr.rd > dist_thresh and mr.bd > dist_thresh):  # No obst
        #     if(intensity > intensity_last):
        #         [v, omega] = bot.run()  # controller run()
        #     else:
        #         [v, omega] = bot.tumble()  # controller tumble()
        # # Paranoid behavior - run away from obstacle
        # else:
        #     # Closest obstacle detected among the 4 directions
        #     mrmin = min(mr.ld, mr.fd, mr.rd, mr.bd)
        #     mrindex = [mr.ld, mr.fd, mr.rd, mr.bd].index(mrmin)
        #     if(mrindex == 0):
        #         obst_pos = mr.lpoint
        #     elif(mrindex == 1):
        #         obst_pos = mr.fpoint
        #     elif(mrindex == 2):
        #         obst_pos = mr.rpoint
        #     else:
        #         obst_pos = mr.bpoint
        #     [v, omega] = bot.avoid_obst(obst_pos)

        # # Update robot pose as per control input and intensity_last
        robot_x += v*dir_x
        robot_y += v*dir_y
        robot_phi = constrain(robot_phi + omega)
        # intensity_last = intensity

        # Update position of obstacles
        # for i in range(num_obsts):
        #     obsts[i].x += int(1.5 * sin(0.02*pg.time.get_ticks()))
        #     obsts[i].y += int(1.5 * sin(0.02*pg.time.get_ticks()))

        # FPS. Print if required
        # clock.tick(300)     # To limit fps, controls speed of the animation
        # fps = (frames*1000)/(pg.time.get_ticks() - ticks)  # calculate fps

        # Update PyGame display
        pg.display.flip()
        # frames+=1


if(__name__ == '__main__'):
    main()
