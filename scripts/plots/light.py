import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import rosbag
from mpl_toolkits import mplot3d
import os
import csv

# First 5 are from (100,600)
# Next 5 are from (100, 100)
# Next 5 are from (600, 600)
# Next 5 are from (600, 100)
path = '../../data/light_simulation/'
fnames = ['rollout_s_0_20200722_13-05-48.npz',
          'rollout_s_0_20200722_13-06-31.npz',
          'rollout_s_0_20200722_13-06-44.npz',
          # 'rollout_s_0_20200722_13-07-03.npz',
          # 'rollout_s_0_20200722_13-07-36.npz',
          'rollout_s_0_20200724_16-08-41.npz',
          'rollout_s_0_20200724_16-09-23.npz',
          'rollout_s_0_20200724_16-09-44.npz',
          # 'rollout_s_0_20200724_16-10-08.npz',
          # 'rollout_s_0_20200724_16-10-29.npz',
          'rollout_s_0_20200724_16-16-47.npz',
          'rollout_s_0_20200724_16-17-19.npz',
          'rollout_s_0_20200724_16-18-02.npz',
          # 'rollout_s_0_20200724_16-18-29.npz',
          # 'rollout_s_0_20200724_16-19-17.npz',
          'rollout_s_0_20200724_16-34-43.npz',
          'rollout_s_0_20200724_16-35-06.npz',
          'rollout_s_0_20200724_16-35-35.npz']
          # 'rollout_s_0_20200724_16-36-12.npz',
          # 'rollout_s_0_20200724_16-36-39.npz']

source = [350, 350]
starts = np.array([[100, 600],
                   [100, 100],
                   [600, 600],
                   [600, 100]])

t_arr = []
action_arr = []
pos_arr = []
dist_arr = []
obstacles_arr = []

for fname in fnames:
    loadfile = np.load(path + fname)
    t_arr.append(loadfile['rollout_t'])
    action_arr.append(loadfile['rollout_action'])
    pos_arr.append(loadfile['rollout_pos'])
    dist_arr.append(loadfile['rollout_dist'])
    obstacles_arr.append(loadfile['obstacles'])

# Plot 1 - dist vs time from simulation data
plt.figure(1)
for i in range(len(t_arr)):
    plt.plot(t_arr[i], dist_arr[i], label='Trajectory ' + str(i+1))
plt.grid()
#plt.legend()
# plt.title('Distance with time in simulation')
plt.xlabel('time (sec)')
plt.ylabel('distance to source (pixels)')
# plt.show()
plt.savefig('../../../img/sim_dist_t.pdf', bbox_inches="tight")
plt.savefig('../../../img/sim_dist_t.png', bbox_inches="tight")
print("Saved ../../../img/sim_dist_t")

# Plot 2 - trajectories from simulation data
fig = plt.figure(2)
ax = fig.add_subplot(1, 1, 1)
# Draw trajectories
for i in range(len(t_arr)):
    plt.plot(pos_arr[i][:, 0], pos_arr[i][:, 1],
             label='Trajectory ' + str(i+1), linewidth=1)
# Draw obstacles
for obstacle in obstacles_arr[0]:
    ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2]))
# Draw source position
plt.plot(source[0], source[1], 'kD', markersize=5)
# Draw start positions
for pos in starts:
    plt.plot(pos[0], pos[1], 'ro', markersize=5)
plt.xlim([0, 700])
plt.ylim([0, 700])
plt.gca().invert_yaxis()
plt.axis('equal')
# plt.title('Trajectories in simulation')
plt.xlabel('x position (pixels)')
plt.ylabel('y position (pixels)')
#plt.legend()
# plt.show()
plt.savefig('../../../img/sim_trajs.pdf', bbox_inches="tight")
plt.savefig('../../../img/sim_trajs.png', bbox_inches="tight")
print("Saved ../../../img/sim_trajs")

# Plot 3 - trajectories from flight data
for bag_no in range(3):
    path = '../../data/light/'
    names = ["2020-03-07-17-18-28.bag", "2020-03-07-20-14-41.bag", "2020-03-07-20-24-16.bag"]
    endtrim = [226, 257, 227]
    bag_name = path + names[bag_no-1]
    bag = rosbag.Bag(bag_name)
    stateX = []
    stateX_t = []
    stateY = []
    stateY_t = []
    action = []
    action_t = []
    intensity = []
    intensity_t = []

    for topic, msg, t in bag.read_messages(topics=['/bcf_state', '/bcf_action', '/bcf_intensity']):
        if '/bcf_state' in topic:
            stateX.append(msg.stateX)
            stateX_t.append(t.to_sec())
            stateY.append(msg.stateY)
            stateY_t.append(t.to_sec())
        if '/bcf_action' in topic:
            action.append(msg.action)
            action_t.append(t.to_sec())
        if '/bcf_intensity' in topic:
            intensity.append(msg.intensity)
            intensity_t.append(t.to_sec())
    stateX = np.asarray(stateX)
    stateX_t = np.asarray(stateX_t)
    stateY = np.asarray(stateY)
    stateY_t = np.asarray(stateY_t)
    action = np.asarray(action)
    action_t = np.asarray(action_t)
    intensity = np.asarray(intensity)
    intensity_t = np.asarray(intensity_t)

    stateX_adj = np.zeros(len(intensity_t))
    stateX_t_adj = np.zeros(len(intensity_t))
    stateY_adj = np.zeros(len(intensity_t))
    stateY_t_adj = np.zeros(len(intensity_t))
    action_adj = np.zeros(len(intensity_t))
    for i in range(len(intensity_t)):
        stateX_adj[i] = stateX[np.argmin(np.abs(stateX_t - intensity_t[i]))]
        stateX_t_adj[i] = stateX_t[np.argmin(np.abs(stateX_t - intensity_t[i]))]
        stateY_adj[i] = stateY[np.argmin(np.abs(stateY_t - intensity_t[i]))]
        stateY_t_adj[i] = stateY_t[np.argmin(np.abs(stateY_t - intensity_t[i]))]

    i = 0
    for j in range(len(action_t)):
        while intensity_t[i] <= action_t[j]:
            action_adj[i] = action[j]
            i += 1
    action_adj[np.where(action_adj == 0)] = 1  

    stateX_adj = stateX_adj[:endtrim[bag_no - 1]]
    stateY_adj = stateY_adj[:endtrim[bag_no - 1]]
    intensity = intensity[:endtrim[bag_no - 1]]
    action_adj = action_adj[:endtrim[bag_no - 1]]
    fig, ax = plt.subplots()

    vmax = 1200
    vmin = 0
    cmap = 'nipy_spectral'
    markersize = 30


    scatter1 = ax.scatter(stateX_adj[action_adj==1], stateY_adj[action_adj==1],
                           marker='.', c=intensity[action_adj==1], label='Run', vmin=vmin, vmax=vmax,
                           cmap=cmap, s=markersize)
    scatter2 = ax.scatter(stateX_adj[action_adj==2], stateY_adj[action_adj==2], 
                           marker='*', c=intensity[action_adj==2], label='Tumble', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=markersize)
    scatter3 = ax.scatter(stateX_adj[action_adj==3], stateY_adj[action_adj==3], 
                           marker='+', c=intensity[action_adj==3], label='Avoid', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=markersize)
    scatterstart = ax.scatter(stateX_adj[0], stateY_adj[0], 
                           marker='x', c='red', label='Start', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=100)
    scatterfinish = ax.scatter(stateX_adj[-1], stateY_adj[-1], 
                           marker='v', c='green', label='Finish', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=100)

    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    if bag_no == 1:
        ax.legend(loc='lower right')
    elif bag_no == 2:
        ax.legend(loc='upper right')
    else:
        ax.legend(loc='upper center')
    ax.axis('equal')
    cb = plt.colorbar(scatter1)
    cb.set_label('Intensity (lux)')

    # ax.set_title('Drone trajectory while seeking light source (trial ' + str(bag_no+1) + ')')
    plt.savefig('../../../img/2d-plot-bag' + str(bag_no+1) +'.pdf', bbox_inches="tight")
    plt.savefig('../../../img/2d-plot-bag' + str(bag_no+1) + '.png', bbox_inches="tight")
    print('Saved ../../../img/2d-plot-bag' + str(bag_no+1))

# Plot 4 - plot for light seeking for fire as a light source
# fig6 = plt.figure(6, figsize=(16, 9), constrained_layout=True)
fig6 = plt.figure(6, figsize=(16, 4))
datadir = '../../data/light/'
trials = os.listdir(datadir)[3:]  # Only take directories
plotted_trials = [1, 2]  # Plot all trials
for trial in trials:
    trial_no = int(trial[-1])
    if trial_no in plotted_trials:
        # Trim data since position estimate is bad during takeoff and land
        if trial_no == 1:
            startTrim = 11
            endTrim = 40
        elif trial_no == 2:
            startTrim = 20
            endTrim = 0
        # ax = plt.subplot(1, 1, 1, title='Trial '+str(trial_no))
        ax = plt.subplot(1, 2, trial_no)

        # position
        x, y, pos_t = [], [], []
        with open(datadir+trial+'/pos.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                x.append(float(line[1]))
                y.append(float(line[2]))
                pos_t.append(int(line[0]))

        # intensity
        intensity, intensity_t = [], []
        with open(datadir+trial+'/intensity.csv') as filehandle:
            csvreader = csv.reader(filehandle)
            for line in csvreader:
                intensity.append(float(line[1]))
                intensity_t.append(int(line[0]))

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
        intensity = np.asarray(intensity)
        intensity_t = np.asarray(intensity_t)
        action = np.asarray(action)
        action_t = np.asarray(action_t)

        # Adjust position for asynchronicity with intensity
        x_adj = np.zeros(len(intensity_t))
        y_adj = np.zeros(len(intensity_t))
        pos_t_adj = np.zeros(len(intensity_t))
        for i in range(len(intensity_t)):
            x_adj[i] = x[np.argmin(np.abs(pos_t -intensity_t[i]))]
            y_adj[i] = y[np.argmin(np.abs(pos_t -intensity_t[i]))]
            pos_t_adj[i] = pos_t[np.argmin(np.abs(pos_t - intensity_t[i]))]

        # Adjust action for asynchronicity with intensity
        action_adj = np.zeros(len(intensity_t))
        i = 0
        for j in range(len(action_t)):
            if i < len(intensity_t)-1:
                while intensity_t[i] <= action_t[j]:
                    action_adj[i] = action[j]
                    i += 1
        action_adj[np.where(action_adj == 0)] = 1  

        # Trim noisy position estimates from trajectory
        x_adj = x_adj[startTrim:len(x_adj)-endTrim]
        y_adj = y_adj[startTrim:len(y_adj)-endTrim]
        intensity = intensity[startTrim:len(intensity)-endTrim]
        action_adj = action_adj[startTrim:len(action_adj)-endTrim]

        cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=min(intensity), vmax=max(intensity)), cmap='nipy_spectral')
        cmap.set_array([])
        ax.scatter(x_adj[action_adj==1], y_adj[action_adj==1], marker='.', c=intensity[action_adj==1], vmin=min(intensity), vmax=max(intensity), s=30, label='Run', zorder=2)
        ax.scatter(x_adj[action_adj==2], y_adj[action_adj==2], marker='*', c=intensity[action_adj==2], vmin=min(intensity), vmax=max(intensity), s=30, label='Tumble', zorder=2)
        ax.scatter(x_adj[action_adj==3], y_adj[action_adj==3], marker='+', c=intensity[action_adj==3], vmin=min(intensity), vmax=max(intensity), s=30, label='Avoid', zorder=2)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1, zorder=1)  # trajectory
        # ax.set_title('Light seeking for fire (Trial ' + str(trial_no) + ')')
        ax.set_xlabel('x position (m)')
        ax.set_ylabel('y position (m)')
        ax.axis('equal')
        plt.subplots_adjust(hspace=0.5)
        # scatter2 = ax.scatter(2, 0, marker='d', color='k')  # show source
        ax.scatter(x_adj[0], y_adj[0], marker='x', c='red', s=100, label='Start')  # start location
        ax.scatter(x_adj[-1], y_adj[-1], marker='v', c='green', s=100, label='Finish')  # end location
        cb = plt.colorbar(cmap)
        cb.set_label(r'Light intensity (lux)')
        # plt.legend([Line2D([0], [0], color='k', lw=0, marker='d')], ['Fire pit'])
        plt.legend()
plt.figure(6)
plt.savefig('../../../img/algo-light-fire.pdf', bbox_inches="tight")
plt.savefig('../../../img/algo-light-fire.png', bbox_inches="tight")
print('Saved ../../../img/algo-light-fire')

# plt.show()
