# Desktop branch
This branch summarize the list of ROS packages and dependencies to install for the desktop computer part of our work. This is a catkin workspace. 

## ROS Packages overview
---
### Main packages

- Monodepth_adabin: Node that generates the depth values from a monocular camera. Custom node inspired by the original implementation of the [AdaBins](https://github.com/shariqfarooq123/AdaBins) neural network.

- rtabmap_ros: SLAM node taken from the official repository of [RTABMap ROS](https://github.com/introlab/rtabmap_ros)

- orb_slam_2_ros: Node taken from the official repository of [ORB-SALM2 ROS](https://github.com/appliedAI-Initiative/orb_slam_2_ros)

- monodepth: First neural network used for this project. Taken from the original GitHub repo of [tentone](https://github.com/tentone/monodepth). The code is written in Python2 and was converted into Python3 for the sake of this work.

### Experimental packages
 
 - opencv_apps: Package that offers a various selection of auxiliary nodes for image processing. Screen capture, feature detection etc. Taken from [ros-perception](https://github.com/ros-perception/opencv_apps) GitHub repository.

- darknet_ros: ROS implementation of YoloV2 and V1, this node was used for image segmentation and object recognition. Originally taken from [leggedrobotics](https://github.com/leggedrobotics/darknet_ros).

- rp_cyclop: Node written in Python3 to project LiDAR scans on a video camera image. Used for LiDAR-Camera calibration. 

- emergency_stop: Node written in Python3 used to prevent the mobile platform from colliding with an obstacle. Takes as input scans from a LiDAR and outputs an emergency message when it detects obstacles that are passed an arbitrary threshold.  Used for demonstration purpose of the advantages of having a LiDAR on a mobile robot. 

- hector_slam: Node used by rtabmap_ros to generate odometry when ICP is turned off. Directly taken from [tu-darmstadt-ros-pkg](https://github.com/tu-darmstadt-ros-pkg/hector_slam)

## Installation
---

### ROS environement
- You need on your machine Ubuntu 20.04 LTS.

- Install ROS Neotic by following the [user guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

- After ROS installation, create a ROS directory, clone the repo in it, select branch Matteo and build the environement with:
    
    >$ catkin_make

### Python3 environement
All the nodes are compiled with python 3.8. Make sure to install the python packages according to your version.

### RTABMap
RTABMap was built from source and requires the user to follow the [install guide](https://github.com/introlab/rtabmap_ros). Make sure to build openCV from source with the non free modules. 

### ORB-SLAM2 
Built from source and the user need to follow the [guide](https://github.com/appliedAI-Initiative/orb_slam_2_ros) on the GitHub repo for additional dependencies. 

### monodepth_adabin

This package requires the installation of torch 1.9 and the addition of a folder "weights" inside the package folder. Inside this folder the user has to download and copy the pre-trained neural network of [AdaBins](https://drive.google.com/drive/folders/1nYyaQXOBjNdUJDsmJpcRpu6oE55aQoLA). The user can chose a model trained on the KITTI (outdoor) or NYU (indoor) dataset.

### monodepth

Requires the installation of tensorflow 2.1.4 and the creation of a folder named "models" in which the user has to put another trained model that can be downloaded [here](https://drive.google.com/drive/folders/1cEEYQ4nLJh84U55qzpFnQwNoiJgc7fcj?usp=sharing). 

### Miscellaneous 

The rest of packages dependencies can be downloaded and installed by following the install guides of each source repository mentioned in [ROS packages overview](#ros-packages-overview)


## How to launch

After you have cloned and compiled the environnement and launched ROS core, just execute the bash file   
    
    >$ bash Ustart.sh

And it will pull the repo, compile the environnement again and give the right permission to python scripts to then launch the specified launch file. If you want to change the launch file, there are multiple versions of it in the launch_work_station folder.  