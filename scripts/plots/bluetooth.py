import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import os
import re
import datetime
import time

datadir = '../../data/rssi/'
offset = 0.3
rates_small = []
rates_dev = []
dists_small = []
dists_dev = []


def get_time_from_timestamp(timestamp):
    return datetime.datetime.strptime(timestamp+'000', "%H:%M:%S.%f")


files = os.listdir(datadir)
for fname in sorted(files):
    board = re.match('^.*_[0-9]', fname).group(0)[:-2]
    dist = int(re.match('.*_[0-9]*m', fname).group(0)[:-1].replace(board+'_', ''))
    point = int(fname[-5])
    with open(datadir+fname, 'r') as filehandle:
        lines = filehandle.readlines()
        starttime = get_time_from_timestamp(lines[0][:12])
        if point == 1:
            rssi_vals = []
            pt1_endtime = get_time_from_timestamp(lines[-1][:12]) - starttime
            dt = get_time_from_timestamp('00:00:00.000') - get_time_from_timestamp('00:00:00.000')
        elif point == 2:
            pt2_endtime = get_time_from_timestamp(lines[-1][:12]) - starttime
            dt = pt1_endtime
        else:
            pt3_endtime = get_time_from_timestamp(lines[-1][:12]) - starttime
            dt = pt2_endtime
        for line in lines:
            t = get_time_from_timestamp(line[:12])
            t = t - starttime + dt
            t = (t.seconds*1e6+t.microseconds)/1e6
            rssi = int(line[16:19])
            rssi_vals.append(rssi)
        rssi_avg = sum(rssi_vals) / len(rssi_vals)
        rssi_max = max(rssi_vals)
        rssi_min = min(rssi_vals)
        rate = len(rssi_vals) / float(t)
        if board == 'small_board':
            marker = 'o'
            dist -= offset
        else:
            marker = 'x'
            dist += offset
        if point == 3:
            if board == 'small_board':
                ecolor = 'red'
            else:
                ecolor = 'blue'
            color = ecolor
            capsize = 5
            linewidth = 2
            markersize = 6
            plt.figure(1)
            plt.errorbar(dist, rssi_avg, yerr=np.array([[rssi_avg - rssi_min], [rssi_max - rssi_avg]]), capsize=capsize, color=color, marker=marker, ecolor=ecolor, label=board+', point '+str(point), linewidth=linewidth, markersize=markersize)
            # plt.text(dist, rssi_avg, str(len(rssi_vals)))  # Annotation for number of packets
            # plt.annotate(str(len(rssi_vals)), (dist,rssi_avg), textcoords='offset points', xytext=(0,10), ha='center')  # Annotation for number of packets
            if board == 'small_board':
                rates_small.append(rate)
                dists_small.append(dist + offset)
            else:
                rates_dev.append(rate)
                dists_dev.append(dist - offset)
# Plot 1 - RSSI variation with distance
plt.figure(1)
plt.legend([Line2D([0], [0], color='blue', lw=linewidth, marker='x'), Line2D([0], [0], color='red', lw=linewidth, marker='o')], ['Dev board', 'Small board'])
# plt.title('RSSI variation with distance')
plt.xlabel('distance (m)')
plt.ylabel('RSSI')
plt.xticks([0, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100])
plt.grid()
plt.savefig('../../../img/BLE_RSSI.pdf', bbox_inches="tight")
plt.savefig('../../../img/BLE_RSSI.png', bbox_inches="tight")
print('Saved ../../../img/BLE_RSSI')

# Plot 2 - packet rate variation with distance
rates_dev = np.roll(np.array(rates_dev), -1)
dists_dev = np.roll(np.array(dists_dev), -1)
plt.figure(2)
plt.legend([Line2D([0], [0], color='blue', lw=linewidth, marker='x'), Line2D([0], [0], color='red', lw=linewidth, marker='o')], ['Dev board', 'Small board'])
plt.plot(dists_small, rates_small, marker='o', color='red')
#plt.text(dists_small, rates_small, str(len(rssi_vals)))  # Annotations for number of packets
plt.plot(dists_dev, rates_dev, marker='x', color='blue')
# plt.title('Packet rate variation with distance')
plt.xlabel('distance (m)')
plt.ylabel('packet rate (packets / sec)')
plt.xticks([0, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100])
plt.grid()
plt.savefig('../../../img/BLE_packet_rate.pdf', bbox_inches="tight")
plt.savefig('../../../img/BLE_packet_rate.png', bbox_inches="tight")
print('Saved ../../../img/BLE_packet_rate.png')

# plt.show()
