# Data Preparation
We provide logs collected from our field experiments as the starting point for running TURTLMap. The logs are collected from two runs at the University of Iowa towing tank. Log1 is the full survey of the tank where the robot is maintained about 1.75m to the bottom. Log2 is the wave run where the robot is maintained at the water surface and drived following a sqaure path. We open-source the logs to facilitate the development of underwater SLAM algorithms.


## Download
The logs are stored in the [ROS bag](http://wiki.ros.org/Bags) format. You can download the logs from the following links:
 - Log1: [link](https://drive.google.com/drive/folders/1WLPOTmcomejHVN03Qs8s_scjXoCne35e?usp=sharing)
 - Log2: [link](https://drive.google.com/drive/folders/1WNZGHc0heD0MtYR1I4XDXcoKkpWlZDqO?usp=sharing)

## Topics
As can be seen, for each log, we provide three separate ros bags, which is due to the convenience of the data collection process. The log1 and log2 are collected in the same way, so we only introduce the topics in log1. For log1, we provide three bags: `log1.bag`, `log1_zed.bag`, and `log1_imu.bag`. The main topics in each bag are as follows:
 - `log1.bag`: 
    - `/dvl/data"`: The measurements of the DVL. We mainly use the `velocity` and the `fom` (figure of merit) in the DVL data.
    - `/BlueROV/pressure2_fluid`: The measurements of the pressure sensor. We use the measured pressure to calculate the depth of the robot.
 - `log1_zed.bag`:
    - `/zedm/zed_node/point_cloud/cloud_registered`: We export the point cloud data from the raw `svo` logs collected by ZED camera for the ease of getting started with TURTLMap. The point cloud data is subscribed by the mapping package (Voxblox) to build the 3D map.
 - `log1_imu.bag`:
    - `/nav/filtered_imu/data`: The IMU data collected by the onboard IMU.

Please also check [this issue](https://github.com/umfieldrobotics/TURTLMap/issues/1) to learn more about how we handle different frame rates of the sensors.

## Use
We have embeeded the playback of the logs in the launch file.

## Ground Truth
We proviode the ground truth of the robot pose in the logs. We collect the ground truth by using the onsite motion capture system. We have converted the mocap log to text files with timestamp and robot pose that can be directly used with the evaluation scripts.