#!/bin/bash

. /opt/ros/groovy/setup.bash
. /opt/ros/hydro/setup.bash

echo "Creating directories..."
mkdir ~/code
mkdir ~/cloudsim/notebook
cd ~/code
mkdir ws
mkdir ws/src
cd ws/src

echo "Transfer repo..."
cp -r ~/cloudsim-ed-actuation ~/code/ws/src
echo "Initializing catkin workspace"
catkin_init_workspace
cd ..

echo "Building catkin workspace"
catkin_make install
. devel/setup.bash
. install/share/cloudsim_ed_actuation/setupx.bash

echo "Webify models for gzweb"
. ~/cloudsim/gzweb/deploy.sh -m local

echo "Setup Notebooks"
cp -r src/cloudsim-ed-actuation/cloudsim_ed_actuation/notebooks/. ~/cloudsim/notebook/

echo "Delete repo files"
rm -r ~/cloudsim-ed-actuation
