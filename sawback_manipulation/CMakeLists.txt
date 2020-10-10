cmake_minimum_required(VERSION 3.0.2)
project(sawback_manipulation)

# add compile options
add_compile_options(-Wall -Wextra)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
	pcl_ros
  roscpp
	rosparam_shortcuts
	sawback_msgs
	sensor_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

find_library(GPD_LIBRARIES NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIBRARIES)
  message(STATUS "Library GPD found in ${GPD_LIBRARIES}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()

find_path(GPD_INCLUDE_DIRS NAMES gpd PATHS /usr/local/include NO_DEFAULT_PATH)
if (GPD_INCLUDE_DIRS)
	message(STATUS "Include directory GPD found in ${GPD_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Include directory GPD not found")
endif()


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 	${PROJECT_NAME}_perception
	${PROJECT_NAME}_solvers
	${PROJECT_NAME}_tasks
 CATKIN_DEPENDS
 	geometry_msgs
	moveit_core
	moveit_ros_planning_interface
	moveit_visual_tools
	roscpp
	sawback_msgs
	sensor_msgs
 DEPENDS
 	Boost
 	EIGEN3
	OpenCV
	PCL
)

include_directories(
	SYSTEM
	${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
	${GPD_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_perception
	src/${PROJECT_NAME}/perception/cloud_processing.cpp
)

add_library(${PROJECT_NAME}_solvers
	src/${PROJECT_NAME}/solvers/cartesian_path.cpp
)

add_library(${PROJECT_NAME}_tasks
	src/${PROJECT_NAME}/tasks/pick_place.cpp
)

# add_dependencies(${PROJECT_NAME}
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )

target_link_libraries(${PROJECT_NAME}_perception
	${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
	${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_tasks
	${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
	${PROJECT_NAME}_solvers
)

target_link_libraries(${PROJECT_NAME}_solvers
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen3_LIBRARIES}
)


add_executable(grasp_detection
	src/grasp_detection_node.cpp
	src/${PROJECT_NAME}/grasp_detection.cpp
)

add_executable(sawback_demo src/sawback_demo_node.cpp)

# add_dependencies(grasp_detection
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
#
# )
#
# add_dependencies(sawback_demo
# 	${${PROJECT_NAME}_EXPORTED_TARGETS}
# 	${catkin_EXPORTED_TARGETS}
# )

target_link_libraries(grasp_detection
	${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
	${GPD_LIBRARIES}
	${PCL_LIBRARIES}
	${PROJECT_NAME}_perception
)

target_link_libraries(sawback_demo
	${catkin_LIBRARIES}
	${Eigen3_LIBRARIES}
	${PROJECT_NAME}_solvers
	${PROJECT_NAME}_tasks
)

# Mark executables for installation
install(TARGETS grasp_detection sawback_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
 		${PROJECT_NAME}_perception
		${PROJECT_NAME}_solvers
		${PROJECT_NAME}_tasks
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
