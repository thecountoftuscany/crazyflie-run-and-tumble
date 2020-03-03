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

def simulate_light_sensor(robot_pos, src_pos, std_dev):
    robot_x = robot_pos[0]
    robot_y = robot_pos[1]
    src_x = src_pos[0]
    src_y = src_pos[1]
    dist = sqrt((robot_x - src_x)**2 + (robot_y - src_y)**2)
    return intensity_scaling/(dist**2) + random.gauss(0, std_dev)


def simulate_rangefinder(robot, obsts):
    xr = robot.x
    yr = robot.y
    phi_r = robot.phi

    dist = []  # Left, front, right, back
    angles = [phi_r - pi/2, phi_r, phi_r + pi/2, phi_r + pi]  # Angles of the 4 sensors
    for phi in angles:
        # Constrain angles within [0, 2pi]
        while(phi < 0):
            phi += 2*pi
        while(phi > 2*pi):
            phi -= 2*pi

        obsts_dists = []  # Will hold distances to all obstacles hit by beam
        for i in range(len(obsts)):
            xo = obsts[i].x
            yo = obsts[i].y
            r = obsts[i].r
            # Intersection of beam with obstacle. Solve beam eqn with obstacle circle eqn
            # Quadratic in x coordinate. Coeffs for ax^2+bx+c=0:
            a = 1 + (tan(phi))**2
            b = 2*(tan(phi)*(yr-xr*tan(phi)) - xo - yo*tan(phi))
            c = (yr-xr*tan(phi))*(yr-xr*tan(phi)-2*yo) + xo**2 + yo**2 - r**2
            if(b**2 - 4*a*c == 0):  # Beam is tangent to obstacle
                # Coords of point where beam hits obstacle
                xhit = -b/(2*a)
                yhit = xhit*tan(phi) + yr - xr*tan(phi)  # From equation of beam
                # Check if obstacle is in front of the sensor
                dot_prod = cos(phi)*(xo-xr) + sin(phi)*(yo-yr)
                if(dot_prod > 0):
                    obsts_dists.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
            elif(b**2 - 4*a*c > 0):  # Beam hits two points on obstacle
                # Coords of points where beam hits obstacle
                xhit1 = (-b + sqrt(b**2-4*a*c))/(2*a)
                xhit2 = (-b - sqrt(b**2-4*a*c))/(2*a)
                yhit1 = xhit1*tan(phi) + yr - xr*tan(phi)  # From equation of beam
                yhit2 = xhit2*tan(phi) + yr - xr*tan(phi)  # From equation of beam
                dist1 = sqrt((xr-xhit1)**2 + (yr-yhit1)**2)
                dist2 = sqrt((xr-xhit2)**2 + (yr-yhit2)**2)
                # Check if obstacle is in front of the sensor
                dot_prod = cos(phi)*(xo-xr) + sin(phi)*(yo-yr)
                if(dot_prod > 0):
                    obsts_dists.append(min(dist1, dist2))  # Choose closer point
        if(len(obsts_dists) > 0):  # Beam hits one or more obstacles
            dist.append(min(obsts_dists))  # Choose closest obstacle
        else:  # Beam hits room walls
            if(phi < pi/2 or phi > 3*pi/2):  # Beam facing right wall
                # Coords of point where beam hits right edge
                xhit = screen_width
                yhit = xhit*tan(phi) + yr - xr*tan(phi)  # From equation of beam
                if(yhit >=0 and yhit <= screen_height):  # Beam hits right edge within room
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
                else:  # Beam hits top or bottom wall
                    if(yhit > screen_height):  # Beam hits bottom wall
                        # Coords of point where beam hits bottom wall
                        yhit = screen_height
                        xhit = xr + (yhit-yr)/(tan(phi))  # From equation of beam
                    else:  # Beam hits top wall
                        # Coords of point where beam hits top wall
                        yhit = 0
                        xhit = xr - yr/tan(phi)  # From equation of beam
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
            elif(phi > pi/2 and phi < 3*pi/2):  # Beam facing left edge
                # Coords of point where beam hits left edge
                xhit = 0
                yhit = yr - xr*tan(phi)  # From equation of beam
                if(yhit >=0 and yhit <= screen_width):  # Beam hits left wall within room
                    dist.append(sqrt((xr-xhit)**2 + (yr-yhit)**2))
                else:  # Beam hits top or bottom wall
                    if(yhit > screen_height):  # Beam hits bottom wall
                        # Coords of point where beam hits bottom wall
                        yhit = screen_height
                        xhit = xr + (yhit-yr)/(tan(phi))  # From equation of beam
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

    def show(self):
        # Left sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x),int(self.y)],
                     [int(self.x+self.ld*cos(self.phi - pi/2)),
                      int(self.y+self.ld*sin(self.phi - pi/2))], 2)
        # Front sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x),int(self.y)],
                     [int(self.x+self.fd*cos(self.phi)),
                      int(self.y+self.fd*sin(self.phi))], 2)
        # Right sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x),int(self.y)],
                     [int(self.x+self.rd*cos(self.phi + pi/2)),
                      int(self.y+self.rd*sin(self.phi + pi/2))], 2)
        # Back sensor
        pg.draw.line(screen, (250, 180, 0), [int(self.x),int(self.y)],
                     [int(self.x+self.bd*cos(self.phi + pi)),
                      int(self.y+self.bd*sin(self.phi + pi))], 2)


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
                if(event.type == pglocs.KEYDOWN and event.key == pglocs.K_SPACE):
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
                       (int(bot.x), int(bot.y)), skirt_r, 0)  # sensor skirt
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
        # Constrain the angle in [0, 2pi]
        while(robot_phi > 2*pi):
            robot_phi -= 2*pi
        while(robot_phi < 0):
            robot_phi += 2*pi
        intensity_last = intensity
        # Update position of obstacles
        # for i in range(num_obsts):
        #     obsts[i].x += int(1.5 * sin(0.02*pg.time.get_ticks()))
        #     obsts[i].y += int(1.5 * sin(0.02*pg.time.get_ticks()))

        # FPS. Print if required
        # clock.tick(300)     # To limit fps, controls speed of the animation
        # fps = (frames*1000)/(pg.time.get_ticks() - ticks)   # calculate current fps

        # Update PyGame display
        pg.display.flip()
        # frames+=1


if(__name__ == '__main__'):
    main()
