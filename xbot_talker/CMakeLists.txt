cmake_minimum_required(VERSION 2.8.3)
project(xbot_talker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  xbot_msgs
  message_generation
)


add_message_files(FILES talk_monitor.msg)
add_service_files(FILES chat.srv play.srv)

generate_messages(
#   DEPENDENCIES
#   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xbot_talker
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE src/linuxrec.cpp src/main.cpp src/speech_recognizer.cpp src/Talker.cpp )

link_directories(/usr/local/lib ${PROJECT_SOURCE_DIR}/lib include ${catkin_INCLUDE_DIRS})

add_executable(xbot_talker ${SOURCE})

target_link_libraries(xbot_talker aiui msc asound pthread  ${catkin_LIBRARIES})

add_dependencies(xbot_talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS xbot_talker
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES lib/libaiui.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES lib/libmsc.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY assets
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


