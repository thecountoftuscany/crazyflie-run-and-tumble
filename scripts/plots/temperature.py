import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator
import numpy as np
import os
import csv


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


# Plot 1 - manual readings
plotted_trials = [2, 3]  # Only plot trials 2, 3
time_trim = 8  # Trim time for all readings to 8 sec
datadir = '../../data/temp/manual/'
trials = os.listdir(datadir)
fig1 = plt.figure(1, figsize=(set_size('ieee-columnwidth', subplots=(2,1))))
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # ax = plt.subplot(2, 1, trial_no-1, title='Trial '+str(trial_no))
        ax = plt.subplot(2, 1, trial_no-1)
        files = os.listdir(datadir+trial)
        for fname in files:
            dist = float(fname[5:8])
            temp_vals = []
            t_vals = []
            with open(datadir+trial+'/'+fname) as filehandle:
                t = 0
                firstline = True
                lines = filehandle.readlines()
                for line in lines:
                    if firstline:
                        firstline = False
                    else:
                        t += 0.200
                    temp = float(line[5:])
                    if t <= 8:
                        temp_vals.append(temp)
                        t_vals.append(t)
                dist_vals = dist * np.ones(len(t_vals))
                ax.plot(t_vals, temp_vals, label=str(dist)+' m')
                #ax.title('Trial ' + str(trial_no))
                # ax.set_title('Temperature variation with ground truth distance (trial ' + str(trial_no) + ')')
                ax.set_xlabel('Time (sec)')
                ax.set_ylabel(r'Temperature ($^\circ$C)')
                ax.grid()
                ax.legend(bbox_to_anchor=(1.04,1.5), loc="upper left")
plt.subplots_adjust(hspace=0.3, right=0.75)
plt.savefig('../../../img/manual-dist-temp.pdf')
plt.savefig('../../../img/manual-dist-temp.png')
print("Saved ../../../img/manual-dist-temp")

datadir = '../../data/temp/cf/'
plotted_trials = [2, 3]  # Only plot trials 2, 3
trials = os.listdir(datadir)
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        fig2 = plt.figure(2, figsize=(set_size('ieee-columnwidth', subplots=(2,1))))
        ax = plt.subplot(2, 1, trial_no-1)
        # Trim data since position estimate is bad during takeoff and land
        if trial_no == 1:
            startTrim = 16
            endTrim = 0
        elif trial_no == 2:
            startTrim = 69
            endTrim = 14
        elif trial_no == 3:
            startTrim = 20
            endTrim = 0

        # position
        x, y, pos_t = [], [], []
        with open(datadir+trial+'/pos.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                x.append(float(line[1]))
                y.append(float(line[2]))
                pos_t.append(int(line[0]))

        # temperature
        temp, temp_t = [], []
        with open(datadir+trial+'/temp.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                temp.append(float(line[1]))
                temp_t.append(int(line[0]))

        x = np.asarray(x)
        y = np.asarray(y)
        pos_t = np.asarray(pos_t)
        temp = np.asarray(temp)
        temp_t = np.asarray(temp_t)

        # Adjust for asynchronicity
        x_adj = np.zeros(len(temp_t))
        y_adj = np.zeros(len(temp_t))
        pos_t_adj = np.zeros(len(temp_t))
        for i in range(len(temp_t)):
            x_adj[i] = x[np.argmin(np.abs(pos_t -temp_t[i]))]
            y_adj[i] = y[np.argmin(np.abs(pos_t -temp_t[i]))]
            pos_t_adj[i] = pos_t[np.argmin(np.abs(pos_t - temp_t[i]))]
        # Trim noisy position estimates from trajectory
        x_adj = x_adj[startTrim:len(x_adj)-endTrim]
        y_adj = y_adj[startTrim:len(y_adj)-endTrim]
        temp = temp[startTrim:len(temp)-endTrim]

        ax.scatter(x_adj, y_adj, marker='.', c=temp, vmin=16, vmax=19.5, s=100)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1)
        # ax.set_title('Temperature variation with drone\'s position estimate (Trial ' + str(trial_no) + ')')
        ax.set_xlabel('x position (m)')
        ax.set_ylabel('y position (m)')
        ax.axis('equal')
        plt.subplots_adjust(hspace=0.5)
        scatter2 = ax.scatter(2, 0, marker='d', color='k')  # show source
        plt.legend([Line2D([0], [0], color='k', lw=0, marker='d')], ['Fire pit'])

        dist = np.sqrt((x_adj-2)**2 + y_adj**2)  # Since the source was at (2,0)
        fig3 = plt.figure(3, figsize=(set_size('ieee-columnwidth', subplots=(2,1))))
        ax = plt.subplot(2, 1, trial_no-1)
        # plt.plot(dist, temp, color='darkgray', linewidth=1)
        ax.scatter(dist, temp, marker='.', s=50)
        # plt.title('Temperature variation with drone-estimated distance (trial ' + str(trial_no) + ')')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel(r'Temperature ($^\circ$C)')
        ax.grid()
        plt.subplots_adjust(hspace=0.5)


# Plot 2 - readings from the crazyflie (traj plot)
plt.figure(2)
fig2.subplots_adjust(right=0.78, left=0.18, hspace=0.5)
cbar_ax = fig2.add_axes([0.82, 0.15, 0.02, 0.7])
cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=16, vmax=19.5), cmap='nipy_spectral')
cmap.set_array([])
cb = plt.colorbar(cmap, cax=cbar_ax)
cb.set_label(r'Temperature ($^\circ$C)')
plt.savefig('../../../img/cf-traj-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/cf-traj-temp.png', bbox_inches="tight")
print('Saved ../../../img/cf-traj-temp')

# Plot 3 - readings from the crazyflie (dist plot)
plt.figure(3)
plt.savefig('../../../img/cf-dist-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/cf-dist-temp.png', bbox_inches="tight")
print('Saved ../../../img/cf-dist-temp')

# Plot 4 - plot for temperature seeking
fig4 = plt.figure(4, figsize=(set_size('ieee-textwidth', subplots=(4,6))))
datadir = '../../data/temp/controller/'
trials = os.listdir(datadir)
plotted_trials = [1, 3, 4, 5, 6]  # Trial 2 doesn't have action data for whatever reason
min_temp, max_temp = 10.5, 13.5
avg_time = 0  # average source seeking time
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # Trim data since position estimate is bad during takeoff and land
        if trial_no == 1:
            startTrim = 75
            endTrim = 70
            placement = fig4.add_gridspec(4, 6)[0:2, 0:2]
        elif trial_no == 2:
            startTrim = 20
            endTrim = 80
            placement = None  # Trial 2 is not plotted
        elif trial_no == 3:
            startTrim = 20
            endTrim = 10
            placement = fig4.add_gridspec(4, 6)[2:4, 0:3]
        elif trial_no == 4:
            startTrim = 20
            endTrim = 10
            placement = fig4.add_gridspec(4, 6)[0:2, 2:4]
        elif trial_no == 5:
            startTrim = 20
            endTrim = 10
            placement = fig4.add_gridspec(4, 6)[0:2, 4:6]
        elif trial_no == 6:
            startTrim = 23
            endTrim = 10
            placement = fig4.add_gridspec(4, 6)[2:4, 3:6]
        # ax = plt.subplot(1, 1, 1, title='Trial '+str(trial_no))
        ax = fig4.add_subplot(placement)

        # position
        x, y, pos_t = [], [], []
        with open(datadir+trial+'/pos.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                x.append(float(line[1]))
                y.append(float(line[2]))
                pos_t.append(int(line[0]))

        # temperature
        temp, temp_t = [], []
        with open(datadir+trial+'/temp.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                temp.append(float(line[1]))
                temp_t.append(int(line[0]))

        # action
        action, action_t = [], []
        with open(datadir+trial+'/action.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                action.append(int(line[1]))
                action_t.append(int(line[0]))

        x = np.asarray(x)
        y = np.asarray(y)
        pos_t = np.asarray(pos_t)
        temp = np.asarray(temp)
        temp_t = np.asarray(temp_t)
        action = np.asarray(action)
        action_t = np.asarray(action_t)

        # Adjust position for asynchronicity with temperature
        x_adj = np.zeros(len(temp_t))
        x_t_adj = np.zeros(len(temp_t))
        y_adj = np.zeros(len(temp_t))
        y_t_adj = np.zeros(len(temp_t))
        pos_t_adj = np.zeros(len(temp_t))
        for i in range(len(temp_t)):
            x_adj[i] = x[np.argmin(np.abs(pos_t -temp_t[i]))]
            y_adj[i] = y[np.argmin(np.abs(pos_t -temp_t[i]))]
            pos_t_adj[i] = pos_t[np.argmin(np.abs(pos_t - temp_t[i]))]

        # Adjust action for asynchronicity with temperature
        action_adj = np.zeros(len(temp_t))
        i = 0
        for j in range(len(action_t)):
            if i < len(temp_t)-1:
                while temp_t[i] <= action_t[j]:
                    action_adj[i] = action[j]
                    i += 1
        action_adj[np.where(action_adj == 0)] = 1  

        # Trim noisy position estimates from trajectory
        # Also, plot every third point since the action (marker types) are indistinguishable otherwise
        skip = 3
        x_adj = x_adj[startTrim:len(x_adj)-endTrim:skip]
        y_adj = y_adj[startTrim:len(y_adj)-endTrim:skip]
        temp = temp[startTrim:len(temp)-endTrim:skip]
        action_adj = action_adj[startTrim:len(action_adj)-endTrim:skip]
        
        avg_time += pos_t_adj[len(pos_t_adj)-endTrim] - pos_t_adj[startTrim]

        ax.scatter(x_adj[action_adj==1], y_adj[action_adj==1], marker='.', c=temp[action_adj==1], vmin=min_temp, vmax=max_temp, s=10, label='Run', zorder=2)
        ax.scatter(x_adj[action_adj==2], y_adj[action_adj==2], marker='*', c=temp[action_adj==2], vmin=min_temp, vmax=max_temp, s=10, label='Tumble', zorder=2)
        ax.scatter(x_adj[action_adj==3], y_adj[action_adj==3], marker='x', c=temp[action_adj==3], vmin=min_temp, vmax=max_temp, s=10, label='Avoid\nObstacle', zorder=2)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1, zorder=1)  # trajectory

        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.axis('equal')
        # ax.set_title('Temperature seeking for fire (Trial ' + str(trial_no) + ')')
        # ax.set_xlabel('x position (m)')
        # ax.set_ylabel('y position (m)')
        ax.scatter(x_adj[0], y_adj[0], marker='1', c='red', s=50, label='Start', zorder=2)  # start location
        ax.scatter(x_adj[-1], y_adj[-1], marker='v', c='green', s=50, label='Finish', zorder=1)  # end location

        ax.xaxis.set_major_locator(MultipleLocator(1))
        ax.yaxis.set_major_locator(MultipleLocator(1))

        ax.xaxis.set_minor_locator(MultipleLocator(0.2))
        ax.yaxis.set_minor_locator(MultipleLocator(0.2))

        ax.grid(b=True, which='major', color='#CCCCCC')
        ax.grid(b=True, which='minor', color='#CCCCCC', linestyle='--', linewidth=0.5)
        ax.set_axisbelow(True)

        # These coordinates are considering the fire as the origin
        obst1_pos = np.array([ [1.1],[0.4] ]); obst1_w, obst1_h = 0.3, 0.15; obst1_angle = 70
        obst2_pos = np.array([ [-0.5],[1.0] ]); obst2_w, obst2_h = 0.25, 0.3; obst2_angle = 50
        obst3_pos = np.array([ [0.8],[1.8] ]); obst3_w, obst3_h = 0.2, 0.3; obst3_angle = 20
        if trial_no == 1:
            theta = 140
            src_pos = np.array([ [0.95], [0.8] ])  # m
        if trial_no == 3:
            theta = 20
            src_pos = np.array([ [1.05], [-1.6] ])  # m
        if trial_no == 4:
            theta = 158
            src_pos = np.array([ [0.95], [1.45] ])  # m
        if trial_no == 5:
            theta = 185
            src_pos = np.array([ [-0.3], [1.5] ])  # m
        if trial_no == 6:
            theta = 100
            src_pos = np.array([ [1.8], [0.3] ])  # m
        rot_mat = np.array([[np.cos(np.pi*theta/180), -np.sin(np.pi*theta/180)], [np.sin(np.pi*theta/180), np.cos(np.pi*theta/180)]])
        obst1 = rot_mat @ (obst1_pos) + src_pos
        obst2 = rot_mat @ (obst2_pos) + src_pos
        obst3 = rot_mat @ (obst3_pos) + src_pos
        # Add source
        ax.scatter(src_pos[0], src_pos[1], marker='d', color='k', s=20, label="Fire pit", zorder=9)
        # Add obstacles
        ax.add_patch(plt.Rectangle(obst1.reshape((2,)), obst1_w, obst1_h, obst1_angle+theta, antialiased=True, color="red", fill=True, label="Obstacles", visible=True, zorder=1))
        ax.add_patch(plt.Rectangle(obst2.reshape((2,)), obst2_w, obst2_h, obst2_angle+theta, antialiased=True, color="red", fill=True, zorder=1))
        ax.add_patch(plt.Rectangle(obst3.reshape((2,)), obst3_w, obst3_h, obst3_angle+theta, antialiased=True, color="red", fill=True, zorder=1))

avg_time /= len(plotted_trials)  # ms
avg_time /= 1000  # s
print('The average seek time for successful temperature seeking trials was {} sec'.format(avg_time))

plt.figure(4)
plt.legend(bbox_to_anchor=(-1.12,-0.06), loc="upper left", ncol=7)
cbar_ax = fig4.add_axes([0.12, -0.12, 0.78, 0.05])
cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=min_temp, vmax=max_temp), cmap='nipy_spectral')
cmap.set_array([])
cb = plt.colorbar(cmap, cax=cbar_ax, orientation="horizontal")
cb.set_label(r'Temperature ($^\circ$C)')
plt.savefig('../../../img/algo-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/algo-temp.png', bbox_inches="tight")
print('Saved ../../../img/algo-temp')

# Plot 5 - combination of the ground truth temp dist plot and the crazyflie measured temp dist plots
fig5 = plt.figure(5, figsize=(set_size(516/3, subplots=(2,1), fraction=0.9)))
ax = plt.subplot(2, 1, 1)
datadir = '../../data/temp/manual/trial3'
files = os.listdir(datadir)
for fname in files:
    dist = float(fname[5:8])
    temp_vals = []
    t_vals = []
    with open(datadir+'/'+fname) as filehandle:
        t = 0
        firstline = True
        lines = filehandle.readlines()
        for line in lines:
            if firstline:
                firstline = False
            else:
                t += 0.200
            temp = float(line[5:])
            if t <= 8:
                temp_vals.append(temp)
                t_vals.append(t)
        dist_vals = dist * np.ones(len(t_vals))
        ax.plot(t_vals, temp_vals, label=str(dist)+' m')
        #ax.title('Trial ' + str(trial_no))
        # ax.set_title('Temperature variation with ground truth distance (trial ' + str(trial_no) + ')')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'Temperature ($^\circ$C)')
        ax.grid()
        ax.legend(bbox_to_anchor=(1.02,1.08), loc="upper left")
ax = plt.subplot(2, 1, 2)
datadir = '../../data/temp/cf/'
plotted_trials = [2, 3]  # Only plot trials 2, 3
trials = os.listdir(datadir)
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # Trim data since position estimate is bad during takeoff and land
        if trial_no == 1:
            startTrim = 16
            endTrim = 0
        elif trial_no == 2:
            startTrim = 69
            endTrim = 14
        elif trial_no == 3:
            startTrim = 20
            endTrim = 0

        # position
        x, y, pos_t = [], [], []
        with open(datadir+trial+'/pos.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                x.append(float(line[1]))
                y.append(float(line[2]))
                pos_t.append(int(line[0]))

        # temperature
        temp, temp_t = [], []
        with open(datadir+trial+'/temp.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                temp.append(float(line[1]))
                temp_t.append(int(line[0]))

        x = np.asarray(x)
        y = np.asarray(y)
        pos_t = np.asarray(pos_t)
        temp = np.asarray(temp)
        temp_t = np.asarray(temp_t)

        # Adjust for asynchronicity
        x_adj = np.zeros(len(temp_t))
        y_adj = np.zeros(len(temp_t))
        pos_t_adj = np.zeros(len(temp_t))
        for i in range(len(temp_t)):
            x_adj[i] = x[np.argmin(np.abs(pos_t -temp_t[i]))]
            y_adj[i] = y[np.argmin(np.abs(pos_t -temp_t[i]))]
            pos_t_adj[i] = pos_t[np.argmin(np.abs(pos_t - temp_t[i]))]
        # Trim noisy position estimates from trajectory
        x_adj = x_adj[startTrim:len(x_adj)-endTrim]
        y_adj = y_adj[startTrim:len(y_adj)-endTrim]
        temp = temp[startTrim:len(temp)-endTrim]

        dist = np.sqrt((x_adj-2)**2 + y_adj**2)  # Since the source was at (2,0)
        # plt.plot(dist, temp, color='darkgray', linewidth=1)
        ax.plot(dist, temp, marker='.', markersize=3, label='Trial'+str(trial_no-1))
        # plt.title('Temperature variation with drone-estimated distance (trial ' + str(trial_no) + ')')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel(r'Temperature ($^\circ$C)')
ax.grid()
ax.legend(bbox_to_anchor=(1.02,0.52), loc="upper left")
plt.subplots_adjust(hspace=0.7, right=0.65, left=0.2, top=0.95, bottom=0.15)
plt.savefig('../../../img/temp-dist.pdf')
plt.savefig('../../../img/temp-dist.png')
print('Saved ../../../img/temp-dist')

# plt.show()
