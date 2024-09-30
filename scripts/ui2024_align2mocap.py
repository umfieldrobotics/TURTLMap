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

save_mocap = True

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


# save mocap to txt
# write to a txt file
# timestamp tx ty tz qx qy qz qw
if save_mocap:
    with open("/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/" + run_name_ours + '_mocap.txt', 'w') as f:
        for i in range(mocap_position_data.shape[2]):
            tx, ty, tz = mocap_position_data[1, :, i]
            if np.isnan(tx) or np.isnan(ty) or np.isnan(tz):
                continue
            rot_matrix = mocap_rotation_data[1, :, i].reshape(3, 3).T
            pose = np.eye(4)
            pose[:3, :3] = rot_matrix
            pose[:3, 3] = [tx, ty, tz]
            rot = R.from_matrix(pose[:3, :3])
            qx, qy, qz, qw = rot.as_quat()
            tx, ty, tz = pose[:3, 3]
            f.write(f"{mocap_time_array[i]} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")


ours_file_name = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/' + run_name_ours + '_dvl_local.txt'

ts_list = []
pose_list = []
# read each line of the file
with open(ours_file_name, 'r') as f:
    lines = f.readlines()
    for line in lines:
        data = line.split()
        ts_list.append(float(data[0]))
        # pose_list.append([float(data[1]), float(data[2]), float(data[3]), float(data[5]), float(data[6]), float(data[7]), float(data[4])])# x, y, z, qx, qy, qz, qw
        pose_list.append([float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[5]), float(data[6]), float(data[7])])# x, y, z, qx, qy, qz, qw

ours_pose_array = np.array(pose_list)
ours_ts_array = np.array(ts_list)


ts_matching_threshold = 0.005
# go through the mocap array and check the 
for i in range(1000, mocap_time_array.shape[0]):
    if not np.isnan(mocap_position_data[1,0,i]):
        # now the pose is nan
        min_ts_diff_pose_idx = np.argmin(np.abs(ours_ts_array - mocap_time_array[i]))
        min_ts_diff = abs(ours_ts_array[min_ts_diff_pose_idx] - mocap_time_array[i])
        if min_ts_diff < ts_matching_threshold and mocap_rotation_data[1,8,i] < -0.995:
            print(f'Found the min i {i}')
            print(f'Mocap ts: {mocap_time_array[i]} and pose ts: {ours_ts_array[min_ts_diff_pose_idx]}')
            min_ts_diff_mocap_idx = i
            break
        # for j in range(ts_array.shape[0]):

init_pose = ours_pose_array[min_ts_diff_pose_idx, :]
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


converted_pose_txt = '/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/' + run_name_ours + '_dvl_local_converted.txt'
with open(converted_pose_txt, 'w') as f:
    for i in range(ours_pose_array.shape[0]):
        T_current_pose = np.eye(4)
        T_current_pose[:3,:3] = R.from_quat(ours_pose_array[i,3:]).as_matrix()
        T_current_pose[:3,3] = ours_pose_array[i,:3]
        T_current_pose = T_offset @ T_current_pose
        quat = R.from_matrix(T_current_pose[:3,:3]).as_quat()
        f.write(f'{ours_ts_array[i]} {T_current_pose[0,3]} {T_current_pose[1,3]} {T_current_pose[2,3]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n')
        
