cmake_minimum_required(VERSION 3.0.2)
project(sawback_manipulation)

# add compile options
add_compile_options(-Wall -Wextra -Wno-psabi)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sawback_manipulation
 CATKIN_DEPENDS
  moveit_ros_perception
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_visual_tools
	roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sawback_demo src/sawback_demo_node.cpp)

add_dependencies(sawback_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sawback_demo
  ${catkin_LIBRARIES}
)

# Mark executables for installation
install(TARGETS sawback_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
