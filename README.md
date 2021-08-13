# Desktop branch
This branch summerize the list of ROS packages and dependencies to install for the desktop computer part of our work. This is a catkin workspace. 

## ROS Packages overview
---
### Main packages

- Monodepth_adabin: Node that generates the depth values from a monocular camera. Custom node inspired by the original implementation of the [AdaBins](https://github.com/shariqfarooq123/AdaBins) neural network.

- rtabmap_ros: SLAM node taken from the official repostory of [RTABMap ROS](https://github.com/introlab/rtabmap_ros)

- orb_slam_2_ros: Node taken from the official repository of [ORB-SALM2 ROS](https://github.com/appliedAI-Initiative/orb_slam_2_ros)

- monodepth: First neural network used for this project. Taken from the original GitHub repo of [tentone](https://github.com/tentone/monodepth). The code is written in Python2 and was converted into Python3 for the sake of this wotk.

### Experimental packages
 
 - opencv_apps: Package that offers a various selection of auxilary nodes for image processing. Screen capture, feature detection etc. Taken from [ros-perception](https://github.com/ros-perception/opencv_apps) GitHub repository.

- darknet_ros: ROS implementation of YoloV2 and V1, this node was used for image segmentation and object recognition. Originally taken from [leggedrobotics](https://github.com/leggedrobotics/darknet_ros).

- rp_cyclop: Node written in Python3 to project LiDAR scans on a video camera image. Used for LiDAR-Camera calibration. 

- emergency_stop: Node written in Python3 used to prevent the mobile platform from colliding with an obstacle. Takes as input scans from a LiDAR and outputs an emergency message when it detects abstacles that are passed an arbitrary threshold.  Used for demonstration purpose of the advantages of having a LiDAR on a mobile robot. 

- hector_slam: Node used by rtabmap_ros to generate odometry when ICP is turned off. Directly taken from [tu-darmstadt-ros-pkg](https://github.com/tu-darmstadt-ros-pkg/hector_slam)

## Installation
---

### ROS environement
- You need on your machine Ubuntu 20.04 LTS.

- Install ROS Neotic by following the [user guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

- After ROS installation, create a ROS directory, clone the repo in it, select branch MAtteo and build the environement with:
    
    >$ catkin_make

### Python3 environement
All the nodes are compiled with python 3.8. Make sure to install the python packages according to your version.

### RTABMap
RTABMap was built from source and requires the user to follow the [install guide](https://github.com/introlab/rtabmap_ros). Make sure to build openCV from source with the non free modules. 




