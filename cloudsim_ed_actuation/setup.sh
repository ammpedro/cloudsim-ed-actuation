
# not needed since we moved plugin target directory into ${CMAKE_INSTALL_PREFIX}/lib/...
# export LD_LIBRARY_PATH=`pwd`/install/lib/scoring_plugins/plugins:$LD_LIBRARY_PATH
# Make kobuki_description and turtlebot_description gazebo models visible to gazebo at runtime
if [ -f /usr/share/gazebo/setup.sh ]; then
  . /usr/share/gazebo/setup.sh
elif [ -f /usr/share/gazebo/setup.sh ]; then
  . /usr/share/gazebo/setup.sh
else
  echo "Warning: failed to find Gazebo's setup.sh.  You will need to source it manually."
fi

export GAZEBO_MODEL_PATH=`rospack find ramp_60`:`rospack find asphalt_plane`:`rospack find construction_cone`:`rospack find basic_robot`:`rospack find map1`:`rospack find gate`:`rospack find gate_number_1`:`rospack find gate_number_2`:`rospack find gate_number_3`:`rospack find gate_number_4`:`rospack find gate_number_5`:$GAZEBO_MODEL_PATH
