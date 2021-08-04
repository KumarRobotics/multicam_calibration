#!/bin/bash
ls -la
src_dir=pwd
ls -latr /opt/ros/
# set up ROS
if [[ -f "/opt/ros/noetic/setup.bash" ]]; then                                              
    source /opt/ros/noetic/setup.bash                                                       
fi                                                                                          
if [[ -f "/opt/ros/kinetic/setup.bash" ]]; then                                             
    source /opt/ros/kinetic/setup.bash                                                      
fi                                                                                          
if [[ -f "/opt/ros/melodic/setup.bash" ]]; then                                             
    source /opt/ros/melodic/setup.bash                                                      
fi

# make workspace
mkdir -p ../catkin_ws/src
cd ../catkin_ws

# clone or link all required packages
git clone https://github.com/catkin/catkin_simple.git src/catkin_simple
git clone --recursive https://github.com/versatran01/apriltag.git src/apriltag
ln -s $src_dir src/multicam_calibration

catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo

catkin build

