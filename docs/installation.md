# TURTLMap Installation
## GTSAM Installation
We use [GTSAM](https://gtsam.org/) to implement the factor graph that gets used for the localization of the robot. Below are the instructions on how to build the version of GTSAM that is shipped with the TURTLMap codebase:

- Download the `GTSAM 4.2` release from the [official GTSAM GitHub Repo](https://github.com/borglab/gtsam/releases/tag/4.2)
```bash
cd ~/Downloads # change to the directory where the zip file was downloaded to

unzip gtsam-4.2
cd gtsam-4.2/

# Build using the instructions provided by the developers
mkdir build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF
make check -j # -j flag to use all available cores. can change by 
sudo make install
```
- After succesfully completing these steps, GTSAM should be installed onto your system.
  - To ensure this, run `ls /usr/local/include/gtsam` and check if the folder exists.


## TURTLMap Installation
Clone TURTLMap into a catkin workspace by using the following instructions:
```bash
mkdir -p ~/turtlmap_ws/src
cd ~/turtlmap_ws/src
git clone [NEED TO ADD REPO]
```
### Installing the required packages
We provide a `.rosinstall` file that contains the required packges to build and run TURTLMap. This can be used as follows:
```bash
cd ~/turtlmap_ws/src
wstool init

# for https
wstool merge onr_posegraph_backend_online/install/turtlmap_ros_https.rosinstall

# for ssh
wstool merge onr_posegraph_backend_online/install/turtlmap_ros_ssh.rosinstall
wstool update
```
This will place all the necessary packages into the `src/` directory of you workspace.

### Compilation
To compile:
```bash
cd ~/turlmap_ws/
catkin build

# Source the environment
source devel/setup.bash
```
If compiling for the first time, it may take ~5 minutes.