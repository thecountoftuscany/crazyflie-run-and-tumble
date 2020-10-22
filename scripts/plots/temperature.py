import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import os
import csv

# Plot 1 - manual readings
plotted_trials = [2, 3]  # Only plot trials 2, 3
time_trim = 8  # Trim time for all readings to 8 sec
datadir = '../../data/temp/manual/'
trials = os.listdir(datadir)
fig1 = plt.figure(1, figsize=(7, 7))
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
                ax.legend(bbox_to_anchor=(1.04,1), loc="upper left", borderaxespad=0)
plt.subplots_adjust(hspace=0.3)
plt.savefig('../../../img/manual-dist-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/manual-dist-temp.png', bbox_inches="tight")
print("Saved ../../../img/manual-dist-temp")

datadir = '../../data/temp/cf/'
plotted_trials = [2, 3]  # Only plot trials 2, 3
trials = os.listdir(datadir)
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # fig2 = fig = plt.figure(trial_no+1)
        fig2 = fig = plt.figure(2)
        # ax = plt.subplot(1, 1, 1, title='Trial '+str(trial_no))
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

        cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=min(temp), vmax=max(temp)), cmap='nipy_spectral')
        cmap.set_array([])
        ax.scatter(x_adj, y_adj, marker='.', c=temp, vmin=min(temp), vmax=max(temp), s=100)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1)
        # ax.set_title('Temperature variation with drone\'s position estimate (Trial ' + str(trial_no) + ')')
        ax.set_xlabel('x position (m)')
        ax.set_ylabel('y position (m)')
        ax.axis('equal')
        plt.subplots_adjust(hspace=0.5)
        scatter2 = ax.scatter(2, 0, marker='d', color='k')  # show source
        cb = plt.colorbar(cmap)
        cb.set_label(r'Temperature ($^\circ$C)')
        plt.legend([Line2D([0], [0], color='k', lw=0, marker='d')], ['Fire pit'])

        dist = np.sqrt((x_adj-2)**2 + y_adj**2)  # Since the source was at (2,0)
        # fig3 = plt.figure(trial_no+5)
        fig3 = plt.figure(3)
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
plt.savefig('../../../img/cf-traj-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/cf-traj-temp.png', bbox_inches="tight")
print('Saved ../../../img/cf-traj-temp')

# Plot 3 - readings from the crazyflie (dist plot)
plt.figure(3)
plt.savefig('../../../img/cf-dist-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/cf-dist-temp.png', bbox_inches="tight")
print('Saved ../../../img/cf-dist-temp')

# Plot 4 - plot for temperature seeking
# fig4 = plt.figure(4, figsize=(16, 9), constrained_layout=True)
fig4 = plt.figure(4, figsize=(16, 9))
gs = fig4.add_gridspec(3, 2)
datadir = '../../data/temp/controller/'
trials = os.listdir(datadir)
plotted_trials = [1, 3, 4, 5, 6]  # Trial 2 doesn't have action data for whatever reason
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # Trim data since position estimate is bad during takeoff and land
        if trial_no == 1:
            startTrim = 75
            endTrim = 80
            placement = gs[0, 0]
        elif trial_no == 2:
            startTrim = 20
            endTrim = 80
            placement = None
        elif trial_no == 3:
            startTrim = 20
            endTrim = 10
            placement = gs[0, 1]
        elif trial_no == 4:
            startTrim = 20
            endTrim = 10
            placement = gs[1, 0]
        elif trial_no == 5:
            startTrim = 20
            endTrim = 10
            placement = gs[1, 1]
        elif trial_no == 6:
            startTrim = 23
            endTrim = 10
            placement = gs[2, :]
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
        y_adj = np.zeros(len(temp_t))
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
        x_adj = x_adj[startTrim:len(x_adj)-endTrim]
        y_adj = y_adj[startTrim:len(y_adj)-endTrim]
        temp = temp[startTrim:len(temp)-endTrim]
        action_adj = action_adj[startTrim:len(action_adj)-endTrim]

        cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=min(temp), vmax=max(temp)), cmap='nipy_spectral')
        cmap.set_array([])
        ax.scatter(x_adj[action_adj==1], y_adj[action_adj==1], marker='.', c=temp[action_adj==1], vmin=min(temp), vmax=max(temp), s=20, label='Run', zorder=2)
        ax.scatter(x_adj[action_adj==2], y_adj[action_adj==2], marker='*', c=temp[action_adj==2], vmin=min(temp), vmax=max(temp), s=20, label='Tumble', zorder=2)
        ax.scatter(x_adj[action_adj==3], y_adj[action_adj==3], marker='+', c=temp[action_adj==3], vmin=min(temp), vmax=max(temp), s=20, label='Avoid', zorder=2)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1, zorder=1)  # trajectory
        # ax.set_title('Temperature seeking for fire (Trial ' + str(trial_no) + ')')
        ax.set_xlabel('x position (m)')
        ax.set_ylabel('y position (m)')
        ax.axis('equal')
        plt.subplots_adjust(hspace=0.5)
        # scatter2 = ax.scatter(2, 0, marker='d', color='k')  # show source
        ax.scatter(x_adj[0], y_adj[0], marker='x', c='red', s=100, label='Start')  # start location
        ax.scatter(x_adj[-1], y_adj[-1], marker='v', c='green', s=100, label='Finish')  # end location
        cb = plt.colorbar(cmap)
        cb.set_label(r'Temperature ($^\circ$C)')
        # plt.legend([Line2D([0], [0], color='k', lw=0, marker='d')], ['Fire pit'])
        plt.legend()
plt.figure(4)
plt.savefig('../../../img/algo-temp.pdf', bbox_inches="tight")
plt.savefig('../../../img/algo-temp.png', bbox_inches="tight")
print('Saved ../../../img/algo-temp')

# plt.show()
