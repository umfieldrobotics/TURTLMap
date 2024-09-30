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
Simply run:
```
roslaunch turtlmap turtlmap.launch
```
