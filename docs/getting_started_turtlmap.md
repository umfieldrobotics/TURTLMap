# Getting Started With TURTLMap
## Getting the Data
You can find the data [on this Google Drive Link](https://drive.google.com/drive/folders/1qnpFIBM9dssAMrt2E_aTrE8YH2lJJM-i). Download the data and change the path to the data in the `launch/turtlmap.launch` file. We note that it's important to start the logs from different timestamps to ensure proper synchronization of the bags that contain different sensor information. The provided data has three rosbags per log:
1. `logX.bag`: this contains all the information that is collected on the BR2 computer, including the **DVL**, **barometer**, and their static transforms.
2. `logX_imu.bag`: this contains the information from the **3DM-GX5-25 IMU**, which we use for the factor graph.
3. `logX_zed-001.bag`: this contains the stereo images, IMU and registered pointcloud from the on-board Zed-Mini camera


### Running for Each Log
Use the `launch/turtlmap.launch file`.
- For log1, set the start flag `-s` in the rosbag player node as **20**
- For log2, set the start flag `-s` in the rosbag player node as **15**

<!-- ### Run command
`rosbag play /frog-drive/bluerov/UI2024/0111/d3_r10_imu.bag /frog-drive/bluerov/UI2024/0111/d3_r10.bag /home/jingyu/frog/zed_test_ws/src/d3_r10_zed_fixed_ts.bag  --clock -s 8 /tf:=/tf_old /tf_static:=/tf_static_old`

`rosbag play /frog-drive/bluerov/UI2024/0113/d4_r13_imu.bag /frog-drive/bluerov/UI2024/0113/d4_r13.bag /home/jingyu/frog/zed_test_ws/src/d4_r13_zed_fixed_ts.bag  --clock -s 9 /tf:=/tf_old /tf_static:=/tf_static_old`

`rosbag play '/home/user/orin/d4_r10.bag' '/home/user/orin/d4_r10_imu.bag' '/home/user/orin/d4_r10_zed_fixed_ts.bag'  --clock /tf:=/tf_old /tf_static:=/tf_static_old -s 15`

`rosbag play '/home/user/orin/d4_r10.bag' '/home/user/orin/d4_r10_imu.bag' '/home/user/orin/d4_r10_zed_fixed_ts.bag'  --clock /tf:=/tf_old /tf_static:=/tf_static_old -s 10`

`rosbag play '/home/user/orin/d4_r13.bag' '/home/user/orin/d4_r13_imu.bag' '/home/user/orin/d4_r13_zed_fixed_ts.bag' --clock /tf:=/tf_old /tf_static:=/tf_static_old -s 20`

`rosbag play '/home/user/orin/d4_r13.bag' '/home/user/orin/d4_r13_imu.bag' '/home/user/orin/d4_r13_zed_fixed_ts.bag' --clock /tf:=/tf_old /tf_static:=/tf_static_old -s 9` -->
