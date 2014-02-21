cmake_minimum_required(VERSION 2.8.3)

project(test_motor)

catkin_package()

#install(DIRECTORY meshes
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_motor
#)
#install(DIRECTORY materials
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_motor
#)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  model.sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_motor
)
