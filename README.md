# Tested on:

1. Ubuntu 12.04 x64 Virtual Machine via VMware Fusion 6.0.2
2. ROS Hydro
3. CloudSim Pre-Release 2.0.1 (simulator-stable)

# How to Deploy on a CloudSim Instance:

## ssh into CloudSim machine
instructions: http://gazebosim.org/wiki/Tutorials/CloudSim/connect_machines

## Clone deploy script
hg clone https://bitbucket.org/hugomatic/cloudsim_ros_deploy
cd cloudsim_ros_deploy

## Run deploy script
./deploy.bash ammpedro cloudsim-ed-actuation

## Use CloudSim Launch Task UI
ROS Package: cloudsim_ed_actuation
Launch File: cloudsim_ed_actuation_challenge_01.launch
Bash File: /home/ubuntu/cloudsim-ed-actuation/src/cloudsim-ed-actuation/setup.bash

#How to install manually on a CloudSim instance

## create catkin workspace
cd /home/ubuntu
mkdir actu
mkdir actu/src
cd actu/src

## get the code
hg clone http://bitbucket.org/ammpedro/cloudsim-ed-actuation

## source ROS
. /opt/ros/groovy/setup.bash
cd /home/ubuntu/actu

## compile
catkin_make install

## source files and publish models to gzweb
. /home/ubuntu/actu/src/cloudsim-ed-actuation/setup.bash
cd /home/ubuntu/cloudsim/gzweb

## webify every model on the local machine
./deploy.sh -m local

## how to start it:
roslaunch cloudsim_ed_actuation cloudsim_ed_actuation_challenge_01.launch
