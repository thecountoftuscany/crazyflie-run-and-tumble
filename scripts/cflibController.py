import argparse
import pygame
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


# uri = 'radio://0/69/2M/E7E7E7E7E7'
uri = 'radio://0/13/2M/E7E7E7E7E7'
is_FlowDeck_attached = True
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Global variables for logging, storing
# position_estimate = [0, 0, 0]

# num_samples = 10
takeoff_height = 0.3    # m
forward_vel = 0.1       # m/s
turn_rate = 360.0 / 10.0 # rot/s
# strafe_vel = 0.1        # m/s
# turn_deg = 20           # deg
# max_intensity = 800   	# lux
# obst_dist_thresh = 500  # mm
# start_time = 5          # sec
# run_time = 1            # sec
# tumble_time = 0.5       # sec
# strafe_time = 2	        # sec
# ao_time = 0.5           # sec


#######################################
# Crazyflie functions
#######################################
def FlowDeckCheck(name, value):
    global is_FlowDeck_attached
    if value:
        is_FlowDeck_attached = True
        print('Flow Deck is attached!')
    else:
        is_FlowDeck_attached = False
        print('Flow Deck is NOT attached!')


def start(scf):
    with MotionCommander(scf, default_height=takeoff_height) as mc:
        # time.sleep(1)
        post_takeoff(mc)


def post_takeoff(mc):
    # rospy.Subscriber('keyinput', String, manual_control_callback)
    pginit(mc)
    # mc.forward(0.5, velocity=forward_vel)
    # mc.back(0.5, velocity=forward_vel)
    # time.sleep(1)


def log_pos_callback(timestamp, data, logconf):
    # if not args.manual_control:
        # print("x={}, y={}, z={}".format(data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    pos_file_handler.write("{},{},{},{}\n".format(timestamp, data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    # global position_estimate
    # position_estimate[0] = data['stateEstimate.x']
    # position_estimate[1] = data['stateEstimate.y']
    # position_estimate[2] = data['stateEstimate.z']


def log_temp_callback(timestamp, data, logconf):
    if not args.manual_control:
        print("temp={}".format(data['HDC2010.temp']))
    temp_file_handler.write("{},{}\n".format(timestamp, data['HDC2010.temp']))
    # global temp
    # temp = data['HDC2010.temp']


def log_range_callback(timestamp, data, logconf):
    # if not args.manual_control:
        # print("rangeLeft={}, rangeFront={}, rangeRight={}, rangeBack={}".format(data['range.left'], data['range.front'], data['range.right'], data['range.back']))
    print(data)
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
    text = titlefont.render('Keyboard input logger for controlling crazyflie', True, (255, 255, 255)); screen.blit(text, (8,10))
    descfont = pygame.font.SysFont('hack', 18)
    text = descfont.render('Keys: w,s,a,d> move;   q,e> turn;   f> stop;   z>land', True, (255, 255, 255)); screen.blit(text, (5, int(win_height/2 - 10)))
    #text = descfont.render('Press Esc to quit.', True, (255, 255, 255)); screen.blit(text, (5,win_height/2 + 10))
    pygame.display.flip()
    # PyGame loop
    while(1):
        # To exit
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        elif event.type == KEYDOWN and event.key == K_w:
            mc.stop()
            mc.start_forward(velocity=forward_vel)
        elif event.type == KEYDOWN and event.key == K_s:
            mc.stop()
            mc.start_back(velocity=forward_vel)
        elif event.type == KEYDOWN and event.key == K_a:
            mc.stop()
            mc.start_left(velocity=forward_vel)
        elif event.type == KEYDOWN and event.key == K_d:
            mc.stop()
            mc.start_right(velocity=forward_vel)
        elif event.type == KEYDOWN and event.key == K_q:
            mc.stop()
            mc.start_turn_left(rate=turn_rate)
        elif event.type == KEYDOWN and event.key == K_e:
            mc.stop()
            mc.start_turn_right(rate=turn_rate)
        elif event.type == KEYDOWN and event.key == K_f:
            mc.stop()
        elif event.type == KEYDOWN and event.key == K_z:
            mc.stop()
            mc.land()
            break
#######################################


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Script for controlling the crazyflie and logging sensor data')
    parser.add_argument('-p', '--log_pos', action='store_true', default=False, help='Log position')
    parser.add_argument('-t', '--log_temp', action='store_true', default=False, help='Log temperature')
    parser.add_argument('-r', '--log_range', action='store_true', default=False, help='Log range')
    parser.add_argument('-c', '--manual_control', action='store_true', default=False, help='Logging only, no flight')
    args = parser.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=os.path.expanduser("~") + "/.cache")) as scf:

        if args.log_pos:
            # Logging position
            # Overwrite logfile contents
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
            # time.sleep(1)

        # Start logging
        if args.log_pos:
            logconf_pos.start()
        if args.log_temp:
            logconf_temp.start()
        if args.log_range:
            logconf_range.start()

        if not args.manual_control:
            while(1):
                try:
                    time.sleep(0.001)
                except KeyboardInterrupt:
                    break
        else:
            if is_FlowDeck_attached:
                # Take off and start
                start(scf)
                        
        # Stop logging and end
        if args.log_pos:
            logconf_pos.stop()
            pos_file_handler.close()
        if args.log_temp:
            logconf_temp.stop()
            temp_file_handler.close()
        if args.log_range:
            logconf_range.stop()
            range_file_handler.close()
