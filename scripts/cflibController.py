# This part gets rid of the 'Hello from PyGame...' message
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame
import argparse
from pygame.locals import *
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
import logging
import time
import sys
import os
import numpy as np

parser = argparse.ArgumentParser(description='Script for controlling the crazyflie, logging sensor data and live plotting position')
parser.add_argument('-p', '--log_pos', action='store_true', default=False, help='Log position')
parser.add_argument('-t', '--log_temp', action='store_true', default=False, help='Log temperature')
parser.add_argument('-r', '--log_range', action='store_true', default=False, help='Log range')
parser.add_argument('-c', '--manual_control', action='store_true', default=False, help='Manually control flying when passed; logging only and no flight if not passed')
parser.add_argument('-l', '--live_plot', action='store_true', default=False, help='Live plot the 3D position')
parser.add_argument('-w', '--write_to_file', action='store_true', default=False, help='Write logging variables to file')
parser.add_argument('-u', '--uri', type=str, default='radio://0/80/2M/E7E7E7E7E7', help='URI of the crazyflie to connect to')
parser.add_argument('-th', '--takeoff_height', type=float, default=0.3, help='Takeoff height of the drone (in m)')
parser.add_argument('-v', '--lin_vel', type=float, default=0.1, help='Linear velocity of the drone (in m/s)')
parser.add_argument('-a', '--ang_vel', type=float, default=360.0/10.0, help='Angular velocity of the drone (in rotations/s)')
parser.add_argument('-d', '--debug_print', action='store_true', default=False, help='Whether to print debug messages to the terminal')
parser.add_argument('-b', '--use_blitting', action='store_true', default=False, help='Whether to use blitting in the live plot.\
        When also controlling the crazyflie, blitting causes faster plotting, but removes dynamically changing axes limits for example.\
        Only applies if both controlling and plotting. If only plotting position by manually moving the crazyflie, non-blitted plots are also as responsive.\
        Hence keep this defaulted to False.\
        Default axes limits to blit are: [-1, 1] for both x and y axes and [0, TAKEOFF_HEIGHT+0.2] for the z axis.')
args = parser.parse_args()

## Some default URIs. Delegated this to the --uri argument with a default
# uri = 'radio://0/69/2M/E7E7E7E7E7'
# uri = 'radio://0/13/2M/E7E7E7E7E7'

## Default axes limits if BLIT. Change these if needed
xlims = [-1, 1]
ylims = [-1, 1]
zlims = [0, args.takeoff_height+0.2]

is_FlowDeck_attached = True
## Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

## Global variables for logging, storing
traj_x, traj_y, traj_z = [0], [0], [0]


#######################################
# Crazyflie functions
#######################################
def FlowDeckCheck(name, value):
    global is_FlowDeck_attached
    if value:
        is_FlowDeck_attached = True
        if args.debug_print:
            print('Flow Deck is attached!')
    else:
        is_FlowDeck_attached = False
        if args.debug_print:
            print('Flow Deck is NOT attached!')


def start(scf):
    with MotionCommander(scf, default_height=args.takeoff_height) as mc:
        post_takeoff(mc)


def post_takeoff(mc):
    pginit(mc)


def log_pos_callback(timestamp, data, logconf):
    if args.debug_print:
        print("x={}, y={}, z={}".format(data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    if args.write_to_file:
        pos_file_handler.write("{},{},{},{}\n".format(timestamp, data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    if args.live_plot:
        global traj_x, traj_y, traj_z
        traj_x.append(data['stateEstimate.x'])
        traj_y.append(data['stateEstimate.y'])
        traj_z.append(data['stateEstimate.z'])
    # global position_estimate
    # position_estimate[0] = data['stateEstimate.x']
    # position_estimate[1] = data['stateEstimate.y']
    # position_estimate[2] = data['stateEstimate.z']


def log_temp_callback(timestamp, data, logconf):
    if args.debug_print:
        print("temp={}".format(data['HDC2010.temp']))
    if args.write_to_file:
        temp_file_handler.write("{},{}\n".format(timestamp, data['HDC2010.temp']))
    # global temp
    # temp = data['HDC2010.temp']


def log_range_callback(timestamp, data, logconf):
    if args.debug_print:
        print("rangeLeft={}, rangeFront={}, rangeRight={}, rangeBack={}".format(data['range.left'], data['range.front'], data['range.right'], data['range.back']))
    if args.write_to_file:
        range_file_handler.write("{},{},{},{},{}\n".format(timestamp, data['range.left'], data['range.front'], data['range.right'], data['range.back']))
    # global rangeLeft, rangeFront, rangeRight, rangeBack
    # rangeLeft, rangeFront, rangeRight, rangeBack = data['range.left'], data['range.front'], data['range.right'], data['range.back']
#######################################


#######################################
# My manual functions
def pginit(mc):
    pygame.init()
    win_width=400
    win_height=400
    screen = pygame.display.set_mode((win_width, win_height), DOUBLEBUF)
    pygame.display.set_caption("Crazyflie control")
    screen.fill((50, 55, 60))  # background
    titlefont = pygame.font.SysFont('hack', 20)
    text = titlefont.render('Crazyflie controller', True, (255, 255, 255)); screen.blit(text, (8,10))
    descfont = pygame.font.SysFont('hack', 18)
    text = descfont.render('Keys: w,s,a,d> move;   q,e> turn;   f> stop;   z>land', True, (255, 255, 255)); screen.blit(text, (5, int(win_height/2 - 10)))
    #text = descfont.render('Press Esc to quit.', True, (255, 255, 255)); screen.blit(text, (5,win_height/2 + 10))
    pygame.display.flip()
    # PyGame loop
    while(1):
        try:
            # To exit
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break
            elif event.type == KEYDOWN and event.key == K_r:
                mc.stop()
                mc.start_linear_motion(1, 1, 0)
            elif event.type == KEYDOWN and event.key == K_w:
                mc.stop()
                mc.start_forward(velocity=args.lin_vel)
            elif event.type == KEYDOWN and event.key == K_s:
                mc.stop()
                mc.start_back(velocity=args.lin_vel)
            elif event.type == KEYDOWN and event.key == K_a:
                mc.stop()
                mc.start_left(velocity=args.lin_vel)
            elif event.type == KEYDOWN and event.key == K_d:
                mc.stop()
                mc.start_right(velocity=args.lin_vel)
            elif event.type == KEYDOWN and event.key == K_q:
                mc.stop()
                mc.start_turn_left(rate=args.ang_vel)
            elif event.type == KEYDOWN and event.key == K_e:
                mc.stop()
                mc.start_turn_right(rate=args.ang_vel)
            elif event.type == KEYDOWN and event.key == K_f:
                mc.stop()
            elif event.type == KEYDOWN and event.key == K_z:
                mc.stop()
                mc.land()
                break
        except KeyboardInterrupt:
            mc.stop()
            mc.land()
            break
#######################################


#######################################
# Live plot
def live_plot(i, fig, ax, line):
    line.set_data(np.array(traj_x), np.array(traj_y))
    line.set_3d_properties(np.array(traj_z))
    if not BLIT:
        if max(traj_x) < 1 and min(traj_x) > -1:
            ax.set_xlim(-1, 1)
        else:
            ax.set_xlim(min(traj_x), max(traj_x))
        if max(traj_y) < 1 and min(traj_y) > -1:
            ax.set_ylim(-1, 1)
        else:
            ax.set_ylim(min(traj_y), max(traj_y))
        if max(traj_z) < args.takeoff_height+0.2:
            ax.set_zlim(0, args.takeoff_height+0.2)
        else:
            ax.set_zlim(0, max(traj_z))
    return line,


def manual_control(scf):
    if is_FlowDeck_attached:
        start(scf)  # Take off and start
#######################################


if __name__=='__main__':

    if args.live_plot and not args.log_pos:
        print('Position logging (-p or --log_pos) compulsary if using live plotting (-l or --live_plot). Check the docstring with -h for more details. Exiting.')
        sys.exit()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=os.path.expanduser("~") + "/.cache")) as scf:

        if args.log_pos:
            # Logging position
            # Overwrite logfile contents
            if args.write_to_file:
                pos_file_handler = open("../data/pos.csv", "w")
                pos_file_handler.close()
                pos_file_handler = open("../data/pos.csv", "a")
            logconf_pos = LogConfig(name='Position', period_in_ms=10)
            logconf_pos.add_variable('stateEstimate.x', 'float')
            logconf_pos.add_variable('stateEstimate.y', 'float')
            logconf_pos.add_variable('stateEstimate.z', 'float')
            scf.cf.log.add_config(logconf_pos)
            logconf_pos.data_received_cb.add_callback(log_pos_callback)

        if args.log_temp:
            # Logging temperature
            # Overwrite logfile contents
            if args.write_to_file:
                temp_file_handler = open("../data/temp.csv", "w")
                temp_file_handler.close()
                temp_file_handler = open("../data/temp.csv", "a")
            logconf_temp = LogConfig(name='Temperature', period_in_ms=200)
            logconf_temp.add_variable('HDC2010.temp', 'float')
            scf.cf.log.add_config(logconf_temp)
            logconf_temp.data_received_cb.add_callback(log_temp_callback)

        if args.log_range:
            # Logging range
            # Overwrite logfile contents
            if args.write_to_file:
                range_file_handler = open("../data/range.csv", "w")
                range_file_handler.close()
                range_file_handler = open("../data/range.csv", "a")
            logconf_range = LogConfig(name='Range', period_in_ms=10)
            logconf_range.add_variable('range.left', 'uint16_t')
            logconf_range.add_variable('range.front', 'uint16_t')
            logconf_range.add_variable('range.back', 'uint16_t')
            logconf_range.add_variable('range.right', 'uint16_t')
            scf.cf.log.add_config(logconf_range)
            logconf_range.data_received_cb.add_callback(log_range_callback)

        # Checks if the FlowDeck is connected
        if args.manual_control:
            scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                    cb=FlowDeckCheck)

        # Start logging
        if not args.live_plot and args.log_pos:
            logconf_pos.start()
        if args.log_temp:
            logconf_temp.start()
        if args.log_range:
            logconf_range.start()

        if args.manual_control and not args.live_plot:
            manual_control(scf)

        if args.live_plot:
            import _thread
            from matplotlib import pyplot as plt
            from matplotlib import animation
            from mpl_toolkits.mplot3d import Axes3D

            fig = plt.figure()
            # ax = fig.gca(projection='3d')
            ax = Axes3D(fig)
            line, = ax.plot([], [], [])
            if BLIT:
                ax.set_xlim(xlims)
                ax.set_ylim(ylims)
                ax.set_zlim(zlims)
            anim_object = animation.FuncAnimation(fig, live_plot, interval=50, blit=BLIT, fargs=(fig, ax, line,))
            if args.manual_control:
                _thread.start_new_thread(manual_control, (scf,))
            if args.log_pos:
                time.sleep(5)
                logconf_pos.start()
            plt.show()

        if not args.manual_control and not args.live_plot:
            while(1):
                try:
                    time.sleep(0.001)
                except KeyboardInterrupt:
                    break
                        
        # Stop logging and end
        if args.log_pos:
            logconf_pos.stop()
            if args.write_to_file:
                pos_file_handler.close()
        if args.log_temp:
            logconf_temp.stop()
            if args.write_to_file:
                temp_file_handler.close()
        if args.log_range:
            logconf_range.stop()
            if args.write_to_file:
                range_file_handler.close()
