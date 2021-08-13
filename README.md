# Raspberry Pi branch
This branch summarize the list of ROS packages and dependencies to install for the RAspberry Pi part of our work. This is a catkin workspace.

## ROS Packages overview
---
### Main packages

- rplidar_ros: Node used to interface with the RPLidar A3 from SLAMTECH. Direcly taken from [robopeak](https://github.com/robopeak/rplidar_ros)

- raspicam_node: Node used to interface with the Raspberry camera V2 and directly taken from [UbiquityRobotics](https://github.com/UbiquityRobotics/raspicam_node)

- mpu_6050_driver: Custom node inspired by [OSUrobotics](https://github.com/OSUrobotics/mpu_6050_driver) to interface with the MPU-6050 IMU. 

### Miscellaneous 
The rest of the packages inside src folder are not used. 

## Installation
---
### ROS environnement
- The Raspberry Pi needs to be on Ubuntu 20.04 LTS 64 bits.
- Install ROS Neotic by following the user [guide](http://wiki.ros.org/noetic/Installation/Ubuntu)
- After ROS installation, create a ROS directory, clone the repo in it, select branch Raspberry and build the environnement with:
    >$ catkin_make
### Python
mpu_6050_node requires Python3 to work. Make sure to fullfil this requirement. 

## How to launch
---
After you have cloned and compiled the environnement and launched ROS core on the remote workstation, just execute the bash file.
    
    >$ bash Ustart.sh

And it will pull the repo, compile the environnement again and give the right permission to python scripts to then launch the specified launch file. If you want to change the launch file, it is located inside the "launch_rpi" folder.