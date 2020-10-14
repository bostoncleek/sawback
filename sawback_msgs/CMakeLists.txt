cmake_minimum_required(VERSION 3.0.2)
project(sawback_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
	geometry_msgs
	message_generation
	std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  SampleGrasps.srv
)

add_action_files( 
  FILES
  PickPlace.action
)

generate_messages(
  DEPENDENCIES
	actionlib_msgs
	geometry_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
  actionlib_msgs
 	geometry_msgs
 	message_runtime
	std_msgs
)

install(DIRECTORY srv/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv)
install(DIRECTORY action/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action)
