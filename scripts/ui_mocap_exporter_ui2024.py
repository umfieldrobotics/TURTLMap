#! /usr/local/bin/python3

# used for exporting the mocap log captured in UI to a rosbag for development

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

def read_mat_file(filename):
    mat = loadmat(filename)
    return mat



# for iowa log 
mat_file_name = '/frog-drive/bluerov/UI2024/UM_WaveBasin_Jan2024/Data/UM_D4_R10.mat'

mat = read_mat_file(mat_file_name)
print(mat.keys())

testrun = mat_file_name.split("/")[-1].split(".")[0]
data = mat['TestRun004']
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

# captured_bag = '/frog-drive/jingyuso/UIowa/new_fixed/3_square_fixed_viz_fluid.bag'
# bag = rosbag.Bag(captured_bag, 'r')
# print(bag.get_start_time())
# time = time.replace(tzinfo=timezone.cst).astimezone(tz=None)


position_data = data['RigidBodies'][0,0]['Positions'][0,0] / 1000.0 # convert to meters
rotation_data = data['RigidBodies'][0,0]['Rotations'][0,0]

# (2, 3, 31159)

fps = 100.0

# write tf message for the 

# we will have to consider the initial pose from SVIn

# Initialized!! As enough keypoints are found
# Initial T_WS: -0.0569452   0.149078   0.987184          0
#   0.149078   0.978973  -0.139239          0
#  -0.987184   0.139239 -0.0779721          0
#          0          0          0          1
# Initial timestamp1674938033.512000083

# first_pose_ts = 1674938033.512000083 #TODO: change this
# first_pose_ts = 1674938022.607000064
first_pose_ts = 1674937528.426073312 # log 3
# first_pose_ts = 1674937976.40703773 # log 4
# find frame number for the first pose
first_pose_frame = int((first_pose_ts - time_unix) * fps)
# get the position and orientation of the first pose in mocap log
first_pose_position = position_data[1,:,first_pose_frame]
first_pose_rotation = rotation_data[1,:,first_pose_frame].reshape(3,3)

first_pose_T = np.eye(4)
first_pose_T[:3,:3] = first_pose_rotation.T
first_pose_T[:3,3] = first_pose_position
print(first_pose_T)

T_expected = np.eye(4)
T_expected[:3,:3] = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
T_expected[:3,3] = first_pose_position

T_set_for_slam = np.linalg.inv(T_expected) @ first_pose_T
print("T_set_for_slam: ", T_set_for_slam)

T_offset = T_expected
# T_offset = np.matmul(T_expected, np.linalg.inv(first_pose_T))

# print("T_offset: ", T_offset)

# TODO: edit this
# VERTEX_SE3_STAMPED 1674938022607000064 0 0.002878752 0.025818497 
# 0.034532048 -0.743101597 0.160472140 0.647119939 0.057311188 0.019831782 
# -0.022550836 -0.009233302 0.088336036 -0.010904213 0.009700766 -0.035755053
# -0.009263406 -0.000125735
# initial_pose = np.array([[-0.0569452, 0.149078, 0.987184, 0],
#                         [0.149078, 0.978973, -0.139239, 0],
#                         [-0.987184, 0.139239, -0.0779721, 0],
#                         [0, 0, 0, 1]])
initial_pose = np.eye(4)
# x: 0.18797445321761863
#       y: -0.03396557420927049
#       z: 0.023306636665692288
# orientation: 
#       x: 0.002582421575484486
#       y: -0.02751314507859955
#       z: -0.06575581123166954
#       w: 0.997453022069689
initial_pose[:3,:3] = scipy.spatial.transform.Rotation.from_quat([0.002582421575484486, -0.02751314507859955, -0.06575581123166954, 0.997453022069689]).as_matrix()
initial_pose[:3,3] = [0.18797445321761863, -0.03396557420927049, 0.023306636665692288]
print("Initial pose: ", initial_pose)

# initial_pose[:3,:3] = scipy.spatial.transform.Rotation.from_quat([-0.743101597, 0.160472140, 0.647119939, 0.057311188]).as_matrix()
# initial_pose[:3,3] = [0.002878752, 0.025818497, 0.034532048]




# - Translation: [-0.135, -0.106, 0.283]
# - Rotation: in Quaternion [0.008, -0.708, 0.013, 0.706]
#             in RPY (radian) [-2.027, -1.562, 2.057]
#             in RPY (degree) [-116.158, -89.508, 117.847]
T_baselink_imu_link = np.eye(4)
# T_baselink_imu_link[:3,:3] = scipy.spatial.transform.Rotation.from_quat([0.008, -0.708, 0.013, 0.706]).as_matrix()
# T_baselink_imu_link[:3,3] = [-0.135, -0.106, 0.283]

# transform_offset = np.matmul(initial_pose, np.linalg.inv(first_pose_T @ T_baselink_imu_link))
# transform_offset = np.eye(4)
# transform_offset

# create TF messages for the poses in a rosbag

# create a new rosbag
bag_to_write = rosbag.Bag('/frog-drive/jingyuso/UIowa/new_fixed/mocap_tf_3_fixed.bag', 'w')
path_msg = Path()
for i in tqdm(range(10, position_data.shape[2])): # we will skip the first 2000 frames 20s to prevent increase the bag playback time
    # get the position and orientation of the first pose in mocap log
    pose_position = position_data[1,:,i]
    pose_rotation = rotation_data[1,:,i]
    # check if there is a NaN value
    
    pose_T = np.eye(4)
    pose_T[:3,:3] = pose_rotation.reshape(3,3).T
    pose_T[:3,3] = pose_position
    # pose_T = np.matmul(transform_offset, pose_T) @ T_baselink_imu_link
    # pose_T[2,3] = -pose_T[2,3] + 1.85101153
    pose_T = np.matmul(np.linalg.inv(T_offset), pose_T)
    tf_msg = TransformStamped()
    tf_msg.header.stamp = rospy.Time.from_sec(time_unix + i/fps)
    tf_msg.header.frame_id = "world"
    tf_msg.child_frame_id = "mocap"
    tf_msg.transform.translation.x = pose_T[0,3]
    tf_msg.transform.translation.y = pose_T[1,3]
    tf_msg.transform.translation.z = pose_T[2,3]
    # if np.isnan(pose_position).any() or np.isnan(pose_rotation).any():
    #     print("NaN value found in mocap log")
    #     print(tf_msg)
    #     break
    # convert rotation matrix to quaternion
    q = scipy.spatial.transform.Rotation.from_matrix(pose_T[:3,:3]).as_quat()
    tf_msg.transform.rotation.x = q[0]
    tf_msg.transform.rotation.y = q[1]
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]
    tf_msg_pub = tfMessage([tf_msg])

    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.from_sec(time_unix + i/fps)
    pose_msg.header.frame_id = "world"
    pose_msg.pose.position.x = pose_T[0,3]
    pose_msg.pose.position.y = pose_T[1,3]
    pose_msg.pose.position.z = pose_T[2,3]
    # convert rotation matrix to quaternion
    q = scipy.spatial.transform.Rotation.from_matrix(pose_T[:3,:3]).as_quat()
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    # add a path every 10 frames
    if i % 10 == 0 and i > first_pose_frame:
        path_msg.header.stamp = rospy.Time.from_sec(time_unix + i/fps)
        path_msg.header.frame_id = "world"
        path_msg.poses.append(pose_msg)
        bag_to_write.write('/mocap_path', path_msg, path_msg.header.stamp)

    bag_to_write.write('/mocap', pose_msg, pose_msg.header.stamp)
    bag_to_write.write('/tf', tf_msg_pub, tf_msg.header.stamp)

bag_to_write.close()

# get bag start time as a time reference to search in mat file