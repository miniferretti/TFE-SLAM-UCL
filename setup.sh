#!/bin/bash
git stash
git pull
cd src/rp_cyclop
chmod +x rp_cyclop.py
cd ..
cd ..
cd src/monodepth/src
chmod +x monodepth.py
cd ..
cd ..
cd ..
cd src/monodepth_adabin/src
chmod +x monodepth_adabin.py
cd ..
cd ..
cd ..
ls
cd src/mpu_6050_driver/scripts
chmod +x imu_node.py
chmod +x tf_broadcaster_imu.py
cd ..
cd ..
cd ..
source devel/setup.bash
echo "Permission successfuly granted and ROS ready to launch"