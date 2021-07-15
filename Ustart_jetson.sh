#!/bin/bash
git pull

source devel/setup.bash 


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
echo "Permission successfuly granted and ready to launch the nodes"

for x in 1 2 3 4 
do
    echo "."
done

roslaunch launch_jetson launch_jetson.launch
