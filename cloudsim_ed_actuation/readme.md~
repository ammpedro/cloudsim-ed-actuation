Tested on:

1. Ubuntu 12.04 x64 Virtual Machine via VMware Fusion 6.0.2
2. ROS Hydro

Notes (01/24/14):

1. Restructured for cloudsim use. Run setup.sh in root directory instead.

Notes (01/22/14):

1. Clone package to catkin_ws/src/cloudsim_ed_actuation_gazebo
2. Setup .bashrc
    - source /usr/share/gazebo-1.9/setup.sh
    - export GAZEBO_MODEL_PATH= /WORKSPACEPATH/catkin_ws/src/cloudsim_ed_actuation_gazebo/models: ${GAZEBO_MODEL_PATH}
    - export GAZEBO_RESOURCE_PATH= /usr/share/gazebo-1.9/worlds: ${GAZEBO_RESOURCE_PATH}
3. Edit local model <uri> to WORKSPACEPATH/catkin_ws/src/cloudsim_ed_actuation_gazebo/models/basic_robot in cloudsim_ed_actuation_gazebo/worlds/cloudsim_ed_actuation_task_01.world
4. Execute catkin_make
5. Test: roslaunch cloudsim_ed_actuation_gazebo cloudsim_ed_actuation_task_01.launch
