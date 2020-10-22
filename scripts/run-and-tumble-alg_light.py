import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
import logging
import time
import sys
import os

from collections import deque
import random
import numpy as np


num_samples = 10
takeoff_height = 0.3    # m
forward_vel = 0.1       # m/s
strafe_vel = 0.1        # m/s
turn_deg = 20           # deg
max_intensity = 800   	# lux
obst_dist_thresh = 500  # mm
run_time = 1            # sec
tumble_time = 0.5       # sec
strafe_time = 1	        # sec
ao_time = 0.5           # sec

# Global variables for logging
x, y, z, t = 0, 0, 0, 0
intensity, last_intensity = 1, 0
rangeLeft, rangeFront, rangeRight, rangeBack = 600, 600, 600, 600
current_action = 1  # 1:run, 2:tumble, 3:avoid-obst

uri = 'radio://0/13/2M/E7E7E7E7E7'
# uri = 'radio://0/69/2M/E7E7E7E7E7'
is_FlowDeck_attached = True
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


#######################################
# Callbacks
#######################################
def log_pos_callback(timestamp, data, logconf):
    # print("x={}, y={}, z={}".format(data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    pos_file_handler.write("{},{},{},{}\n".format(timestamp, data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']))
    global x, y, z, t
    x, y, z, t = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'], timestamp


def log_intensity_callback(timestamp, data, logconf):
    # print("intensity={}, lastintensity={}".format(data['BH1750.intensity'], last_intensity))
    intensity_file_handler.write("{},{}\n".format(timestamp, data['BH1750.intensity']))
    global intensity, d_intensity, last_intensity
    intensity = data['BH1750.intensity']
    d_intensity.append(intensity)
    last_intensity = sum(d_intensity)/len(d_intensity)


def log_range_callback(timestamp, data, logconf):
    # print("rangeLeft={}, rangeFront={}, rangeRight={}, rangeBack={}".format(data['range.left'], data['range.front'], data['range.right'], data['range.back']))
    range_file_handler.write("{},{},{},{},{}\n".format(timestamp, data['range.left'], data['range.front'], data['range.right'], data['range.back']))
    global rangeLeft, rangeFront, rangeRight, rangeBack
    rangeLeft, rangeFront, rangeRight, rangeBack = data['range.left'], data['range.front'], data['range.right'], data['range.back']
#######################################


#######################################
# Functions
#######################################
def FlowDeckCheck(name, value):
    global is_FlowDeck_attached
    if value:
        is_FlowDeck_attached = True
        print('Flow Deck is attached!')
    else:
        is_FlowDeck_attached = False
        print('Flow Deck is NOT attached!')


def run(mc):
    global current_action
    current_action = 1
    action_file_handler.write("{}, {}\n".format(t, current_action))
    mc.start_forward(forward_vel)
    print('Running!')
    time.sleep(run_time)


def tumble(mc):
    global current_action
    current_action = 2
    action_file_handler.write("{}, {}".format(t, current_action))
    # mc.wait()
    c = random.random()
    print('Tumbling! c = ', c)
    if c <= 0.5:
        # pass
        # turns CF in the range 0-180 deg
        mc.turn_left(180*random.random())
        # mc.wait()
    else:
        # pass
        mc.turn_right(180*random.random())
        # mc.wait()
    mc.start_forward(forward_vel)
    time.sleep(tumble_time)


def avoid_obst(mc):
    global current_action, rangeLeft, rangeFront, rangeRight, rangeBack
    current_action = 3
    action_file_handler.write("{}, {}".format(t, current_action))
    dist_arr = np.array([rangeLeft, rangeFront,
                         rangeRight, rangeBack])
    smallest_dist = np.argmin(dist_arr)

    if smallest_dist == 0:
        print('Obstacle to left, moving right')
        # client.wait()
        mc.start_right(strafe_vel)
        time.sleep(strafe_time)
        # mc.wait()
        mc.turn_right(turn_deg)
        # mc.wait()
        mc.start_forward(forward_vel)
    elif smallest_dist == 1:
        print('Obstacle to front, moving back')
        # mc.wait()
        mc.start_back(strafe_vel)
        time.sleep(strafe_time)
        # mc.wait()
        mc.turn_left(turn_deg)
        # mc.wait()
        mc.start_forward(forward_vel)
    elif smallest_dist == 2:
        print('Obstacle to right, moving left')
        # mc.wait()
        mc.start_left(strafe_vel)
        time.sleep(strafe_time)
        # mc.wait()
        mc.turn_left(turn_deg)
        # mc.wait()
        mc.start_forward(forward_vel)
    elif smallest_dist == 3:
        print('Obstacle to back, moving forward')
        # mc.wait()
        mc.start_forward(forward_vel)
    time.sleep(ao_time)
#######################################


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    d_intensity = deque(maxlen=num_samples)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=os.path.expanduser("~") + "/.cache")) as scf:
        # Overwrite logfile contents
        pos_file_handler = open("../data/pos.csv", "w")
        pos_file_handler.close()
        pos_file_handler = open("../data/pos.csv", "a")

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

        # Logging intensity
        # Overwrite logfile contents
        intensity_file_handler = open("../data/intensity.csv", "w")
        intensity_file_handler.close()
        intensity_file_handler = open("../data/intensity.csv", "a")
        logconf_intensity = LogConfig(name='Intensity', period_in_ms=200)
        logconf_intensity.add_variable('BH1750.intensity', 'float')
        scf.cf.log.add_config(logconf_intensity)
        logconf_intensity.data_received_cb.add_callback(log_intensity_callback)

        # Logging range
        # Overwrite logfile contents
        range_file_handler = open("../data/range.csv", "w")
        range_file_handler.close()
        range_file_handler = open("../data/range.csv", "a")
        logconf_range = LogConfig(name='Range', period_in_ms=10)
        logconf_range.add_variable('range.left', 'float')
        logconf_range.add_variable('range.front', 'float')
        logconf_range.add_variable('range.back', 'float')
        logconf_range.add_variable('range.right', 'float')
        scf.cf.log.add_config(logconf_range)
        logconf_range.data_received_cb.add_callback(log_range_callback)

        # Logging action
        # Overwrite logfile contents
        action_file_handler = open("../data/action.csv", "w")
        action_file_handler.close()
        action_file_handler = open("../data/action.csv", "a")

        # Checks if the FlowDeck is connected
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                cb=FlowDeckCheck)
        # time.sleep(1)


        if is_FlowDeck_attached:
            # Start logging
            logconf_pos.start()
            logconf_intensity.start()
            logconf_range.start()

            # Take off and start
            with MotionCommander(scf, default_height=takeoff_height) as mc:
                # Start high level control loop
                while intensity < max_intensity:
                    try:
                        if (rangeLeft > obst_dist_thresh and
                           rangeRight > obst_dist_thresh and
                           rangeFront > obst_dist_thresh and
                           rangeBack > obst_dist_thresh):
                            print("intensity={}, last_intensity={}".format(intensity, last_intensity))
                            if intensity > last_intensity:
                                run(mc)
                            else:
                                tumble(mc)
                        else:
                            avoid_obst(mc)
                    except KeyboardInterrupt:
                        break

                # end while loop
                mc.stop()
                mc.land()
                        
                # Stop logging and end
                logconf_pos.stop()
                pos_file_handler.close()
                logconf_intensity.stop()
                intensity_file_handler.close()
                logconf_range.stop()
                range_file_handler.close()
                action_file_handler.close()
                sys.exit(0)
