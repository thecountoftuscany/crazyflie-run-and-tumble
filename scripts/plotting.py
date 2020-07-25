import numpy as np
import matplotlib.pyplot as plt

# First 5 are from (100,600)
# Next 5 are from (100, 100)
# Next 5 are from (600, 600)
# Next 5 are from (600, 100)
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
    loadfile = np.load(fname)
    t_arr.append(loadfile['rollout_t'])
    action_arr.append(loadfile['rollout_action'])
    pos_arr.append(loadfile['rollout_pos'])
    dist_arr.append(loadfile['rollout_dist'])
    obstacles_arr.append(loadfile['obstacles'])

# Plot 1 - dist vs time
plt.figure(1)
for i in range(len(t_arr)):
    plt.plot(t_arr[i], dist_arr[i], label='Trajectory ' + str(i+1))
plt.grid()
#plt.legend()
plt.xlabel('time (sec)')
plt.ylabel('distance to source (pixels)')
plt.show()

# Plot 2 - trajectories
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
plt.xlabel('x position (pixels)')
plt.ylabel('y position (pixels)')
#plt.legend()
plt.show()
