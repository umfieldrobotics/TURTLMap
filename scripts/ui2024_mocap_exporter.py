import rospy
import rosbag
import os
import sys
import numpy as np
from scipy.io import loadmat
from datetime import datetime
from datetime import timedelta
from datetime import timezone
from geometry_msgs.msg import TransformStamped
import tf
from tqdm import tqdm
import scipy
from scipy import spatial
from tf.msg import tfMessage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation as R

def read_g2o(file_name):
    vertices = {}
    edges = []

    with open(file_name, 'r') as f:
        for line in f:
            data = line.split()

            if data[0] == "VERTEX_SE3:QUAT":
                id = int(data[1])
                x, y, z = map(float, data[2:5])
                vertices[id] = (x, y, z)

            elif data[0] == "EDGE_SE3:QUAT":
                from_id, to_id = map(int, data[1:3])
                dx, dy, dz = map(float, data[3:6])
                edges.append((from_id, to_id, dx, dy, dz))

    return vertices, edges


def read_mat_file(filename):
    mat = loadmat(filename)
    return mat

run_name = 'UM_D4_R13'
run_name_ours = 'd4_r13'
mat_file_name = '/frog-drive/bluerov/UI2024/UM_WaveBasin_Jan2024/Data/' + run_name + '.mat'
mat = read_mat_file(mat_file_name)
print(mat.keys())

testrun = mat_file_name.split("/")[-1].split(".")[0]
data = mat[run_name]
starting_time = str(data['Timestamp'][0,0][0])
starting_time = starting_time.split("\t")
starting_time = [t.replace(",", "") for t in starting_time]
# dt = datetime.datetime.fromtimestamp(starting_time[0])
time = datetime.strptime(starting_time[0], "%Y-%m-%d %H:%M:%S.%f")# 2023-01-28, 14:24:52.655	20721.32750
# time aware of time zone

# set time zone to central standard time GMT-6 with timedelta
time = time + timedelta(hours=1) # i think our laptop was not adjusted to central time so there is a 1-hour difference

time_unix = time.timestamp()
print(time_unix)

mocap_position_data = data['RigidBodies'][0,0]['Positions'][0,0] / 1000.0 # convert to meters
mocap_rotation_data = data['RigidBodies'][0,0]['Rotations'][0,0]

fps = 100.0

# write a timestamp array
mocap_time_array = np.arange(mocap_position_data.shape[2]) / fps + time_unix

print(mocap_time_array)

# (2, 3, 31159)



# import matplotlib.pyplot as plt

# fig = plt.figure()
# ax = plt.axes(projection='3d')

# ax.plot3D(position_data[1,0,:], position_data[1,1,:], position_data[1,2,:])
# plt.show()

# load the pose and timestamp
# file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs/' + run_name_ours + '_latest.txt'
file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/' + run_name_ours + '_0311_ours.txt'
# file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/' + run_name_ours + '_ukf.txt'



ts_list = []
pose_list = []
# read each line of the file
with open(file_name, 'r') as f:
    lines = f.readlines()
    for line in lines:
        data = line.split()
        ts_list.append(float(data[0]))
        pose_list.append([float(data[1]), float(data[2]), float(data[3]), float(data[5]), float(data[6]), float(data[7]), float(data[4])])# x, y, z, qx, qy, qz, qw
        # pose_list.append([float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[5]), float(data[6]), float(data[7])])# x, y, z, qx, qy, qz, qw

pose_array = np.array(pose_list)
ts_array = np.array(ts_list)

# no_imu_file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs/' + run_name_ours + '_latest_0_1_0deg_0310_fix.txt'
# no_imu_file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs/' + run_name_ours + '_no_imuFactor.txt'
# no_imu_file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs/' + run_name_ours + '_latest_0_1_0deg.txt'
no_imu_file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/' + run_name_ours + '_ukf.txt'

no_imu_ts_list = []
no_imu_pose_list = []
# read each line of the file
with open(no_imu_file_name, 'r') as f:
    lines = f.readlines()
    for line in lines:
        data = line.split()
        no_imu_ts_list.append(float(data[0]))
        no_imu_pose_list.append([float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[5]), float(data[6]), float(data[7])])# x, y, z, qx, qy, qz, qw

no_imu_pose_array = np.array(no_imu_pose_list)
no_imu_ts_array = np.array(no_imu_ts_list)

ts_matching_threshold = 0.005
# go through the mocap array and check the 
for i in range(1000, mocap_time_array.shape[0]):
    if not np.isnan(mocap_position_data[1,0,i]):
        # now the pose is nan
        min_ts_diff_pose_idx = np.argmin(np.abs(ts_array - mocap_time_array[i]))
        min_ts_diff = abs(ts_array[min_ts_diff_pose_idx] - mocap_time_array[i])
        if min_ts_diff < ts_matching_threshold and mocap_rotation_data[1,8,i] < -0.995:
            print(f'Found the min i {i}')
            print(f'Mocap ts: {mocap_time_array[i]} and pose ts: {ts_array[min_ts_diff_pose_idx]}')
            min_ts_diff_mocap_idx = i
            break
        # for j in range(ts_array.shape[0]):

init_pose = pose_array[min_ts_diff_pose_idx, :]
mocap_init_t = mocap_position_data[1,:,min_ts_diff_mocap_idx]
mocap_init_r = mocap_rotation_data[1,:,min_ts_diff_mocap_idx]

print(mocap_init_r.shape)
print(init_pose.shape)

T_init = np.eye(4)
T_init[:3,:3] = R.from_quat(init_pose[3:]).as_matrix()
T_init[:3,3] = init_pose[:3]

print(T_init)

T_init_mocap = np.eye(4)
T_init_mocap[:3,:3] = mocap_init_r.reshape(3,3).T
T_init_mocap[:3,3] = mocap_init_t
print(T_init_mocap)

T_offset = T_init_mocap @ np.linalg.inv(T_init)
T_offset[2,:2] = 0
T_offset[:2,2] = 0
T_offset[2,2] = -1
# normalize the x and y axis
T_offset[:2,0] = T_offset[:2,0] / np.linalg.norm(T_offset[:2,0])
T_offset[:2,1] = T_offset[:2,1] / np.linalg.norm(T_offset[:2,1])

# T_offset[2,2] = -1
T_offset[2,3] = T_init_mocap[2,3] + T_init[2,3]
print("T_offset")
print(T_offset)

mocap_pose = np.vstack((mocap_position_data[1,:,:], np.ones(mocap_position_data.shape[2])))
print(mocap_pose[:,min_ts_diff_mocap_idx])


# mocap_pose_offsets = T_offset @ mocap_pose
# print(mocap_pose_offsets[:,min_ts_diff_mocap_idx])

mocap_pose_offsets = mocap_pose

pose_array = pose_array[:, :3]
pose_array = np.vstack((pose_array.T, np.ones(pose_array.shape[0]))).T
# print(pose_array[min_ts_diff_pose_idx,:])
pose_array = pose_array @ T_offset.T
# print(pose_array[min_ts_diff_pose_idx,:])

ts_matching_threshold = 0.005
# go through the mocap array and check the 
for i in range(1000, mocap_time_array.shape[0]):
    if not np.isnan(mocap_position_data[1,0,i]):
        # now the pose is nan
        min_ts_diff_pose_idx = np.argmin(np.abs(no_imu_ts_array - mocap_time_array[i]))
        min_ts_diff = abs(no_imu_ts_array[min_ts_diff_pose_idx] - mocap_time_array[i])
        if min_ts_diff < ts_matching_threshold and mocap_rotation_data[1,8,i] < -0.995:
            print(f'Found the min i {i}')
            print(f'Mocap ts: {mocap_time_array[i]} and pose ts: {no_imu_ts_array[min_ts_diff_pose_idx]}')
            min_ts_diff_mocap_idx = i
            break
        # for j in range(ts_array.shape[0]):

init_pose = no_imu_pose_array[min_ts_diff_pose_idx, :]
mocap_init_t = mocap_position_data[1,:,min_ts_diff_mocap_idx]
mocap_init_r = mocap_rotation_data[1,:,min_ts_diff_mocap_idx]

print(mocap_init_r.shape)
print(init_pose.shape)

T_init = np.eye(4)
T_init[:3,:3] = R.from_quat(init_pose[3:]).as_matrix()
T_init[:3,3] = init_pose[:3]

print(T_init)

T_init_mocap = np.eye(4)
T_init_mocap[:3,:3] = mocap_init_r.reshape(3,3).T
T_init_mocap[:3,3] = mocap_init_t
print(T_init_mocap)

T_offset = T_init_mocap @ np.linalg.inv(T_init)
T_offset[2,:2] = 0
T_offset[:2,2] = 0
T_offset[2,2] = -1
# normalize the x and y axis
T_offset[:2,0] = T_offset[:2,0] / np.linalg.norm(T_offset[:2,0])
T_offset[:2,1] = T_offset[:2,1] / np.linalg.norm(T_offset[:2,1])

# T_offset[2,2] = -1
T_offset[2,3] = T_init_mocap[2,3] + T_init[2,3]
print("T_offset")
print(T_offset)

no_imu_pose_array = no_imu_pose_array[:, :3]
no_imu_pose_array = np.vstack((no_imu_pose_array.T, np.ones(no_imu_pose_array.shape[0]))).T
no_imu_pose_array = no_imu_pose_array @ T_offset.T


import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot3D(mocap_pose_offsets[0,:], mocap_pose_offsets[1,:], mocap_pose_offsets[2,:],  color='g', label='mocap', linewidth=3, linestyle='dashed')
ax.plot3D(pose_array[:,0], pose_array[:,1], pose_array[:,2], color='r', label='ours')
ax.scatter3D(mocap_pose_offsets[0,min_ts_diff_mocap_idx], mocap_pose_offsets[1,min_ts_diff_mocap_idx], mocap_pose_offsets[2,min_ts_diff_mocap_idx], color='c', label='starting', marker='o')
ax.plot3D(no_imu_pose_array[:,0], no_imu_pose_array[:,1], no_imu_pose_array[:,2], color='b', label='ours_no_imu')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Mocap vs Ours')
# set z range
# ax.set_zlim(-1, 1)

plt.legend()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(mocap_time_array, mocap_pose_offsets[0,:], color='g', label='mocap', linestyle='dashed', linewidth=4)
ax.plot(ts_array, pose_array[:,0], color='r', label='ours')
ax.plot(no_imu_ts_array, no_imu_pose_array[:,0], color='b', label='ours_no_imu')
ax.set_xlabel('Time')
ax.set_ylabel('X')
ax.set_title('X')
plt.legend()


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(mocap_time_array, mocap_pose_offsets[1,:], color='g', label='mocap', linestyle='dashed', linewidth=4)
ax.plot(ts_array, pose_array[:,1], color='r', label='ours')
ax.plot(no_imu_ts_array, no_imu_pose_array[:,1], color='b', label='ours_no_imu')
ax.set_xlabel('Time')
ax.set_ylabel('Y')
ax.set_title('Y')
plt.legend()


fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(mocap_time_array, mocap_pose_offsets[2,:], color='g', label='mocap', linestyle='dashed', linewidth=4)
ax.plot(ts_array, pose_array[:,2], color='r', label='ours')
ax.plot(no_imu_ts_array, no_imu_pose_array[:,2], color='b', label='ours_no_imu')
ax.set_xlabel('Time')
ax.set_ylabel('Z')
ax.set_title('Z')



plt.legend()
plt.show()

