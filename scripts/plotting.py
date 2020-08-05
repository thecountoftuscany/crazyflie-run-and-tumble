import numpy as np
import matplotlib.pyplot as plt
import rosbag
from mpl_toolkits import mplot3d
import tf

# First 5 are from (100,600)
# Next 5 are from (100, 100)
# Next 5 are from (600, 600)
# Next 5 are from (600, 100)
path = '../data/'
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
plt.xlabel('time (sec)')
plt.ylabel('distance to source (pixels)')
plt.show()

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
plt.xlabel('x position (pixels)')
plt.ylabel('y position (pixels)')
#plt.legend()
plt.show()

# Plot 3 - trajectories from flight data
bag_no = 3
names = ["2020-03-07-17-18-28.bag", "2020-03-07-20-14-41.bag", "./2020-03-07-20-24-16.bag"]
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

# plt.savefig('2d-plot-bag' + str(bag_no) +'.eps', format='eps')
# plt.savefig('2d-plot-bag' + str(bag_no) + '.png')
plt.show()
