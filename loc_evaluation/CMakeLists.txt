cmake_minimum_required(VERSION 2.8.3)
project(loc_evaluation)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hmm_measurements
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(loc_evaluation
  src/measurenode.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(loc_evaluation
  ${catkin_LIBRARIES}
)
