Tested on:

1. Ubuntu 12.04 x64 Virtual Machine via VMware Fusion 6.0.2
2. ROS Hydro
3. Cloudsim 1.7.3 (simulator-stable)

Notes (1/24/2014)

1. Clone repo
2. Run setup.sh


How to install on a CloudSim instance

cd /home/ubuntu
mkdir actu
mkdir actu/src
cd actu/src

# get the code
hg clone http://bitbucket.org/ammpedro/cloudsim-ed-actuation

# source ROS
. /opt/ros/groovy/setup.bash
cd /home/ubuntu/actu
# compile
catkin_make_install

# publish models to gzweb
. /home/ubuntu/actu/src/cloudsim-ed-actuation/setup.bash
cd /home/ubuntu/cloudsim/gzweb
# webify every model on the local machine
./deploy.sh -m local

# how to start it:
roslaunch cloudsim_ed_actuation cloudsim_ed_actuation_challenge_01.launch
