import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import os
import re
import datetime
import time

# Optional (but use consistent styles for all plots)
# plt.style.use('seaborn')

# According to the IEEE format
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    "font.size": 10,
    "legend.fontsize": 8,  # verify
    "xtick.labelsize": 8,  # verify
    "ytick.labelsize": 8,  # verify
    "axes.labelsize": 10})

datadir = '../../data/rssi/'
offset = 0.3
rates_small = []
rates_dev = []
dists_small = []
dists_dev = []


def get_time_from_timestamp(timestamp):
    return datetime.datetime.strptime(timestamp+'000', "%H:%M:%S.%f")


def set_size(width, fraction=1, subplots=(1, 1)):
    """Set figure dimensions to avoid scaling in LaTeX.
    Parameters
    ----------
    width: float or string
            Document width in points, or string of predined document type
    fraction: float, optional
            Fraction of the width which you wish the figure to occupy
    subplots: array-like, optional
            The number of rows and columns of subplots.
    Returns
    -------
    fig_dim: tuple
            Dimensions of figure in inches
    """
    if width == 'ieee-textwidth':
        width_pt = 516
    elif width == 'ieee-columnwidth':
        width_pt = 252
    else:
        width_pt = width
    # Width of figure (in pts)
    fig_width_pt = width_pt * fraction
    # Convert from pt to inches
    inches_per_pt = 1 / 72.27
    # Golden ratio to set aesthetic figure height
    # https://disq.us/p/2940ij3
    golden_ratio = (5**.5 - 1) / 2
    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])
    return (fig_width_in, fig_height_in)


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
                ecolor = '#ff7f0e'  # matplotlib's default color#2 (orange-ish)
            else:
                ecolor = '#1f77b4'  # matplotlib's default color#1 (blue-ish)
            color = ecolor
            capsize = 3
            linewidth = 1
            markersize = 3
            width = 'third-of-textwidth'
            if width == 'columnwidth':
                plt.figure(1, figsize=set_size('ieee-columnwidth'))
            else:
                plt.figure(1, figsize=set_size(516/3, fraction=0.9))  # For textwidth/3 width
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
# plt.title('RSSI variation with distance')
plt.xlabel('Distance (m)')
plt.ylabel('RSSI')
if width == 'columnwidth':
    plt.xticks([0, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100])
else:
    plt.xticks([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
plt.grid()
if width == 'columnwidth':
    plt.legend([Line2D([0], [0], color='#1f77b4', lw=linewidth, marker='x'), Line2D([0], [0], color='#ff7f0e', lw=linewidth, marker='o')], ['Dev board', 'Small board'])
    plt.savefig('../../../img/BLE_RSSI.pdf', bbox_inches="tight")
    plt.savefig('../../../img/BLE_RSSI.png', bbox_inches="tight")
    print('Saved ../../../img/BLE_RSSI')
else:
    plt.subplots_adjust(left=0.25, bottom=0.3)
    plt.legend([Line2D([0], [0], color='#1f77b4', lw=linewidth, marker='x', markersize=markersize), Line2D([0], [0], color='#ff7f0e', lw=linewidth, marker='o', markersize=markersize)], ['Large antenna', 'Small antenna'], bbox_to_anchor=(0.3,0.6), loc="lower left")
    plt.savefig('../../../img/BLE_RSSI-resized.pdf')
    plt.savefig('../../../img/BLE_RSSI-resized.png')
    print('Saved ../../../img/BLE_RSSI-resized')

# Plot 2 - packet rate variation with distance
rates_dev = np.roll(np.array(rates_dev), -1)
dists_dev = np.roll(np.array(dists_dev), -1)
if width == 'columnwidth':
    plt.figure(2, figsize=set_size('ieee-columnwidth'))
else:
    plt.figure(2, figsize=set_size(516/3, fraction=0.9, subplots=(2,1)))  # For textwidth/3 width
    plt.subplots_adjust(left=0.23, top=0.95, bottom=0.15, right=0.95)
    # plt.subplots_adjust(left=0.25, bottom=0.3)
    # plt.figure(2, figsize=(214/72.27, 264/72.27))  # For textwidth/3 width and height same as temp dist plot
plt.legend([Line2D([0], [0], color='#1f77b4', marker='x', markersize=markersize, linewidth=linewidth), Line2D([0], [0], color='#ff7f0e', marker='o', markersize=markersize)], ['Large antenna', 'Small antenna'])
plt.plot(dists_small, rates_small, marker='o', color='#ff7f0e', markersize=markersize, linewidth=linewidth)
#plt.text(dists_small, rates_small, str(len(rssi_vals)))  # Annotations for number of packets
plt.plot(dists_dev, rates_dev, marker='x', color='#1f77b4', markersize=markersize, linewidth=linewidth)
# plt.title('Packet rate variation with distance')
plt.xlabel('Distance (m)')
plt.ylabel('Rate (packets / s)')
if width == 'columnwidth':
    plt.xticks([0, 5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100])
else:
    plt.xticks([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
plt.grid()
if width == 'columnwidth':
    plt.savefig('../../../img/BLE_packet_rate.pdf', bbox_inches="tight")
    plt.savefig('../../../img/BLE_packet_rate.png', bbox_inches="tight")
    print('Saved ../../../img/BLE_packet_rate')
else:
    plt.savefig('../../../img/BLE_packet_rate-resized.pdf')
    plt.savefig('../../../img/BLE_packet_rate-resized.png')
    print('Saved ../../../img/BLE_packet_rate-resized')

# plt.show()
