cmake_minimum_required(VERSION 2.8.3)

project(ramp_15)

catkin_package()

#install(DIRECTORY meshes
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ramp_60
#)
#install(DIRECTORY materials
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ramp_60
#)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  model.sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ramp_15
)
