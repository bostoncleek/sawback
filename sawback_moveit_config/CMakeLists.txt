cmake_minimum_required(VERSION 3.0.2)
project(sawback_moveit_config)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY srdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf)
