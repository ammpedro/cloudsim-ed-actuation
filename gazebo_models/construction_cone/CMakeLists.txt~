cmake_minimum_required(VERSION 2.8.3)

project(construction_cone)

catkin_package()

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/construction_cone
)
install(DIRECTORY materials
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/construction_cone
)

# install below for gazebo to find this model through GAZEBO_MODEL_PATH.
install(FILES
  model.config
  model.sdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/construction_cone
)
