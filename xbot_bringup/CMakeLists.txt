cmake_minimum_required(VERSION 2.8.3)
project(xbot_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  message_generation
)

# add_message_files(
#    DIRECTORY msg
#    FILES

# )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs visualization_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES xbot_bringup
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs visualization_msgs
  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


