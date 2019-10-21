cmake_minimum_required(VERSION 2.8.3)
project(xbot_tools)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  xbot_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xbot_tools
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(kalman_filter src/kalman_filter.cpp)
add_dependencies(kalman_filter xbot_msgs_gencpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})

install(TARGETS kalman_filter
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY udev
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/const_dist_move.py scripts/forward.py scripts/gps_code.txt scripts/keyboard_teleop.py scripts/make_plan.py scripts/pcb_checker.py scripts/ppt.png scripts/pub_topic.py scripts/pycamera.py scripts/q4torpy.py scripts/read_serial.py scripts/rotate.py scripts/test.py scripts/xbot_checker.py scripts/xbot_control_view.py scripts/xbot_cruise.py scripts/xbot_qt.ui scripts/xbot_ui.py scripts/xbot_ui.pyc
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


