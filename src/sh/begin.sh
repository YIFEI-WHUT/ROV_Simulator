#! /bin/bash
#根据系统启动相应版本的roscore
#gnome-terminal -x bash -c "" &  #启动roscore
#sleep 2 #等待roscore启动(可以不写)
gnome-terminal -x bash -c "source /opt/ros/melodic/setup.bash;cd /home/ubuntu/catkin_ws;source devel/setup.bash;roslaunch start_simulation start_simulation_without_seabed.launch" 

wait
exit 0
