# Desktop branch
This branch summerize the list of ROS packages and dependencies to install for the desktop computer part of our work. 

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