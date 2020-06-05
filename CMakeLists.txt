cmake_minimum_required(VERSION 2.8.3)

project(hardware_gzb)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  #std_msgs
  serial_arm_lib
  #ipab_lwr_msgs	# it might be trajectory_msgs and sensor_msgs or iiwa_msgs
  iiwa_msgs
  sensor_msgs
  urdf
  kdl_parser
  copilotROSInterface
  underwater_sensor_msgs
  trajectory_msgs
  #roslib
  #orocos_kdl
)

#find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuka gamma stewart falcon ur3
  CATKIN_DEPENDS roscpp message_runtime iiwa_msgs sensor_msgs copilotROSInterface  underwater_sensor_msgs trajectory_msgs control_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Define a library target called kuka
add_library(kuka src/kuka.cpp)
target_link_libraries(kuka ${catkin_LIBRARIES})

# Define a library target called gamma
add_library(gamma src/gamma.cpp)
target_link_libraries(gamma ${catkin_LIBRARIES})

# Define a library target called stewart
add_library(stewart src/stewart.cpp)
target_link_libraries(stewart ${catkin_LIBRARIES})

# Define a library target called falcon
add_library(falcon src/falcon.cpp)
target_link_libraries(falcon ${catkin_LIBRARIES})

# Define a library target called ur3
add_library(ur3 src/ur3.cpp)
target_link_libraries(ur3 ${catkin_LIBRARIES})
