cmake_minimum_required(VERSION 2.8.3)
project(xbot_keyop)
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time xbot_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES xbot
   CATKIN_DEPENDS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time xbot_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h*")

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
