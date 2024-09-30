from scipy.io import loadmat
from datetime import datetime
from datetime import timedelta
import numpy as np
from scipy.spatial.transform import Rotation as R


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

# write to a txt file
# timestamp tx ty tz qx qy qz qw
with open("/home/jingyu/frog/onr_slam_ws/src/rpg_trajectory_evaluation/logs/d4_r13/stamped_groundtruth.txt", 'w') as f:
    for i in range(mocap_position_data.shape[2]):
        tx, ty, tz = mocap_position_data[1, :, i]
        if np.isnan(tx) or np.isnan(ty) or np.isnan(tz):
            continue
        rot_matrix = mocap_rotation_data[1, :, i].reshape(3, 3).T
        pose = np.eye(4)
        pose[:3, :3] = rot_matrix
        pose[:3, 3] = [tx, ty, tz]
        offset = np.eye(4)
        offset[2,2] = -1
        pose = np.dot(offset, pose)
        rot = R.from_matrix(pose[:3, :3])
        qx, qy, qz, qw = rot.as_quat()
        tx, ty, tz = pose[:3, 3]
        f.write(f"{mocap_time_array[i]} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")