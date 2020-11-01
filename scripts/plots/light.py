import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator
import rosbag
from mpl_toolkits import mplot3d
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

width = "third-textwidth"  # either "third-textwidth" or "columnwidth"

# Plot 1 - dist vs time from simulation data
if width == "third-textwidth":
    fig1 = plt.figure(1, figsize=set_size(516/3, fraction=0.9))
else:
    fig1 = plt.figure(1, figsize=(set_size('ieee-columnwidth')))
for i in range(len(t_arr)):
    plt.plot(t_arr[i], dist_arr[i], label='Trajectory ' + str(i+1))
plt.grid()
#plt.legend()
# plt.title('Distance with time in simulation')
plt.xlabel('Time (s)')
plt.ylabel('Distance to source\n(pixels)')
plt.subplots_adjust(left=0.28, bottom=0.28, right=0.95, top=0.95)
# plt.show()
if width == "third-textwidth":
    plt.savefig('../../../img/sim_dist_t-resized.pdf')
    plt.savefig('../../../img/sim_dist_t-resized.png')
    print("Saved ../../../img/sim_dist_t-resized")
else:
    plt.savefig('../../../img/sim_dist_t.pdf', bbox_inches="tight")
    plt.savefig('../../../img/sim_dist_t.png', bbox_inches="tight")
    print("Saved ../../../img/sim_dist_t")

# Plot 2 - trajectories from simulation data
if width == "third-textwidth":
    fig2 = plt.figure(2, figsize=set_size(516/3, fraction=0.8))
else:
    fig2 = plt.figure(2, figsize=(set_size('ieee-columnwidth')))
ax = fig2.add_subplot(1, 1, 1)
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
if width == "third-textwidth":
    plt.savefig('../../../img/sim_trajs-resized.pdf', bbox_inches="tight")
    plt.savefig('../../../img/sim_trajs-resized.png', bbox_inches="tight")
    print("Saved ../../../img/sim_trajs-resized")
else:
    plt.savefig('../../../img/sim_trajs.pdf', bbox_inches="tight")
    plt.savefig('../../../img/sim_trajs.png', bbox_inches="tight")
    print("Saved ../../../img/sim_trajs")

# Plot 3 - trajectories from flight data
# fig3 = plt.figure(figsize=(set_size('ieee-textwidth', subplots=(2,2))))
avg_time = 0  # average seeking time for all successful trials
fig3 = plt.figure(figsize=(set_size('ieee-textwidth', subplots=(1,3))))
gs = fig3.add_gridspec(2, 2)
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

    # Plotting every other point since the actions (marker types) are indistinguishable otherwise
    stateX_adj = stateX_adj[:endtrim[bag_no - 1]:2]
    stateY_adj = stateY_adj[:endtrim[bag_no - 1]:2]
    intensity = intensity[:endtrim[bag_no - 1]:2]
    action_adj = action_adj[:endtrim[bag_no - 1]:2]

    avg_time += stateX_t_adj[endtrim[bag_no-1]] - stateX_t_adj[0]

    vmax = 1200
    vmin = 0
    cmap = 'nipy_spectral'
    markersize = 10

    obst_width, obst_height = 0.45, 0.25  # m
    if bag_no+1 == 1:
        ax = plt.subplot(1, 3, 1)
        # Add source
        ax.scatter(2.2, -0.9, marker='d', color='k', s=20, label="Light source", zorder=9)
        # Add obstacles
        ax.add_patch(plt.Rectangle((0.25,0.2), obst_width, obst_height, -20, antialiased=True, color="red", fill=True, label="Obstacles", visible=True, zorder=1))
        ax.add_patch(plt.Rectangle((0.5,-1.5), obst_width, obst_height, 10, antialiased=True, color="red", fill=True, zorder=1))
        ax.add_patch(plt.Rectangle((1.99,-1.5), 1.45, 0.05, 70, antialiased=True, color="red", fill=True, zorder=1))  # Wall
    elif bag_no+1 == 2:
        ax = plt.subplot(1, 3, 2)
        # Add source
        ax.scatter(-0.45, -1.25, marker='d', color='k', s=20, label="Light source", zorder=9)
        # Add obstacles
        ax.add_patch(plt.Rectangle((0.98,-1.8), obst_width, obst_height, 85, antialiased=True, color="red", fill=True, label="Obstacles", visible=True, zorder=1))
        ax.add_patch(plt.Rectangle((1.5,-0.2), obst_width, obst_height, 140, antialiased=True, color="red", fill=True, zorder=1))
        ax.add_patch(plt.Rectangle((0.01,-2.15), 2.0, 0.05, 115, antialiased=True, color="red", fill=True, zorder=1))  # Wall
    elif bag_no+1 == 3:
        ax = plt.subplot(1, 3, 3)
        # Add source
        ax.scatter(2.5, -1.42, marker='d', color='k', s=20, label="Light source", zorder=9)
        # Add obstacles
        ax.add_patch(plt.Rectangle((1.2,-1.95), obst_width, obst_height, 155, antialiased=True, color="red", fill=True, label="Obstacles", visible=True, zorder=1))
        ax.add_patch(plt.Rectangle((2.1,-0.5), obst_width, obst_height, 95, antialiased=True, color="red", fill=True, zorder=1))
        ax.add_patch(plt.Rectangle((1.65,-2.3), 2.0, 0.05, 45, antialiased=True, color="red", fill=True, zorder=1))  # Wall
    scatter1 = ax.scatter(stateX_adj[action_adj==1], stateY_adj[action_adj==1],
                           marker='.', c=intensity[action_adj==1], label='Run', vmin=vmin, vmax=vmax,
                           cmap=cmap, s=markersize, zorder=2)
    scatter2 = ax.scatter(stateX_adj[action_adj==2], stateY_adj[action_adj==2], 
                           marker='*', c=intensity[action_adj==2], label='Tumble', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=markersize, zorder=2)
    scatter3 = ax.scatter(stateX_adj[action_adj==3], stateY_adj[action_adj==3], 
                           marker='x', c=intensity[action_adj==3], label='Avoid\nObstacle', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=markersize, zorder=2)
    scatterstart = ax.scatter(stateX_adj[0], stateY_adj[0], 
                           marker='1', c='red', label='Start', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=50, zorder=2)
    scatterfinish = ax.scatter(stateX_adj[-1], stateY_adj[-1], 
                           marker='v', c='green', label='Finish', vmin=vmin, vmax=vmax, 
                           cmap=cmap, s=50, zorder=1)
    ax.plot(stateX_adj, stateY_adj, color='darkgray', linewidth=1, zorder=1)  # trajectory

    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))

    ax.xaxis.set_minor_locator(MultipleLocator(0.2))
    ax.yaxis.set_minor_locator(MultipleLocator(0.2))

    ax.grid(b=True, which='major', color='#CCCCCC')
    ax.grid(b=True, which='minor', color='#CCCCCC', linestyle='--', linewidth=0.5)
    ax.set_axisbelow(True)

    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.axis('equal')
    # ax.set_title('Drone trajectory while seeking light source (trial ' + str(bag_no+1) + ')')
    # ax.set_xlabel('x position (m)')
    # ax.set_ylabel('y position (m)')
avg_time /= len(names)
print('The average seek time for successful light seeking trials was {} sec'.format(avg_time))

# fig3.subplots_adjust(right=0.82, hspace=0.3, wspace=0.3)
plt.legend(bbox_to_anchor=(-2.22, -0.1), ncol=7, loc="upper left")
fig3.subplots_adjust(left=0.05, wspace=0.1)
cbar_ax = fig3.add_axes([0.048, -0.35, 0.85, 0.1])
cb = plt.colorbar(scatter1, cax=cbar_ax, orientation="horizontal")
cb.set_label('Intensity (lux)')

plt.savefig('../../../img/algo-light.pdf', bbox_inches="tight")
plt.savefig('../../../img/algo-light.png', bbox_inches="tight")
print('Saved ../../../img/algo-light')

# Plot 4 - plot for light seeking for fire as a light source
# fig6 = plt.figure(6, figsize=(16, 9), constrained_layout=True)
fig6 = plt.figure(6, figsize=(set_size('ieee-textwidth', subplots=(1, 2))))
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

        ax.scatter(x_adj[action_adj==1], y_adj[action_adj==1], marker='.', c=intensity[action_adj==1], vmin=0, vmax=300, s=20, label='Run', zorder=2)
        ax.scatter(x_adj[action_adj==2], y_adj[action_adj==2], marker='*', c=intensity[action_adj==2], vmin=0, vmax=300, s=20, label='Tumble', zorder=2)
        ax.scatter(x_adj[action_adj==3], y_adj[action_adj==3], marker='+', c=intensity[action_adj==3], vmin=0, vmax=300, s=20, label='Avoid\nObstacle', zorder=2)
        ax.plot(x_adj, y_adj, color='darkgray', linewidth=1, zorder=1)  # trajectory
        # ax.set_title('Light seeking for fire (Trial ' + str(trial_no) + ')')
        plt.xlabel('x position (m)')
        plt.ylabel('y position (m)')
        if trial_no == 1:
            plt.legend()
        ax.axis('equal')
        # scatter2 = ax.scatter(2, 0, marker='d', color='k')  # show source
        ax.scatter(x_adj[0], y_adj[0], marker='x', c='red', s=100, label='Start')  # start location
        ax.scatter(x_adj[-1], y_adj[-1], marker='v', c='green', s=100, label='Finish')  # end location
        # plt.legend([Line2D([0], [0], color='k', lw=0, marker='d')], ['Fire pit'])
plt.figure(6)
fig6.subplots_adjust(right=0.82)
cbar_ax = fig6.add_axes([0.85, 0.15, 0.02, 0.7])
cmap = matplotlib.cm.ScalarMappable(norm=plt.Normalize(vmin=0, vmax=300), cmap='nipy_spectral')
cmap.set_array([])
cb = plt.colorbar(cmap, cax=cbar_ax)
cb.set_label(r'Light intensity (lux)')
plt.savefig('../../../img/algo-light-fire.pdf', bbox_inches="tight")
plt.savefig('../../../img/algo-light-fire.png', bbox_inches="tight")
print('Saved ../../../img/algo-light-fire')

# plt.show()
