# TFE-SLAM-UCL
---
 Git repo associated to the Master Thesis "LIDAR - Camera fusion for SLAM of autonomous robot or drones" by Matteo Ferretti di Castelferretto and Bruno Nicoletta. 
The manuscript of this master thesis can be consulted at the following address : https://drive.google.com/file/d/1AOqGBLV8_cL5rh4tKFNnJwdQ2t4KcyXo/view?usp=sharing

This repository is the source of all our work and contains a ROS workspace dedicated to the exploration of different technics of integrating a 2D LiDAR to a drone. Visual summary of the work:
 - A custom mobile platform: ![alt text](https://github.com/miniferretti/TFE-SLAM-UCL/blob/main/images/mobilePlat_crop.jpg)
 - CAD model: ![alt text](https://github.com/miniferretti/TFE-SLAM-UCL/blob/main/images/Vue-cavali%C3%A8re.png)
 - ROS implementation diagram: ![alt text](https://github.com/miniferretti/TFE-SLAM-UCL/blob/main/images/ROSFlowChart.png)
 - Working example (rp_cyclop node, projection of lidar data on the video stream): ![alt text](https://github.com/miniferretti/TFE-SLAM-UCL/blob/main/LiDAR_Camera_Matching%20videos/cyclop.gif)
 - Depth extraction from video stream with LiDAR data: 
### Branches dispatching and explanation
---
The Git is divided into multiple branches but only two are usefully. Namely: 
- WorkStation: Branch containing the code that has to run on a desktop computer.
- Raspberry: Branch containing the code that has to run on a Raspberry Pi. 

Each of these branches has a dedicated Readme for installation procedure. 

