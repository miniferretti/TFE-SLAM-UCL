#!/bin/bash
git pull

source devel/setup.bash 


cd src/rp_cyclop
chmod +x rp_cyclop.py

echo "Permission successfuly granted and ready to launch the nodes"

for x in 1 2 3 4 
do
    echo "."
done

roslaunch launch_work_station launch_work_station_Cyclop_only.launch