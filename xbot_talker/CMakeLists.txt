cmake_minimum_required(VERSION 2.8.3)
project(xbot_talker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_C_STANDARD 99)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
find_package(curl COMPONENTS)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    dialogue.msg
    dialogue_ros.msg
    action.msg
    action_ros.msg
    recog_result.msg
    online_asr_result.msg
    awaken_status.msg
    actionmode.msg
    actionmode_ros.msg)

## Generate services in the 'srv' folder
add_service_files(FILES
  history_record.srv
  roolback_dialogue.srv
  deploy_dialogue.srv
  add_dialogue.srv
  call_action.srv
  XbotTts.srv
  call_asr.srv
  call_actionmode.srv
  nlpdialog_config.srv
  keyword_config.srv
  EnableASR.srv
  chat.srv
  play.srv
  CallChat.srv
  CallPlay.srv
  CallVersion.srv

  )


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)



## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xbot_talker
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS system_lib 
)


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


link_directories(/usr/local/lib ${PROJECT_SOURCE_DIR}/lib include ${catkin_INCLUDE_DIRS})


file(GLOB SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/library/*.cpp")

## Declare a C++ executable
add_executable(asr_sample src/ros/asr_module.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})
add_executable(nlp_sample src/ros/nlp_module.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})
add_executable(tts_sample src/ros/tts_module.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})
add_executable(awaken_sample src/ros/awaken_module.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})

add_executable(read_files_asr tests/asr_test/read_all_files_and_recog.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})

add_executable(tuling_test tests/nlp_test/tuling_test.cpp src/ros/xbot_talker_ros.cpp ${SOURCE})



## Specify libraries to link a library or executable target against
target_link_libraries(asr_sample msc asound pthread  curl ${catkin_LIBRARIES})
target_link_libraries(nlp_sample msc asound pthread  curl ${catkin_LIBRARIES})
target_link_libraries(tts_sample msc asound pthread  curl ${catkin_LIBRARIES})
target_link_libraries(awaken_sample msc asound pthread  curl ${catkin_LIBRARIES})


target_link_libraries(read_files_asr msc asound pthread curl ${catkin_LIBRARIES})
target_link_libraries(tuling_test msc asound pthread  curl ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
add_dependencies(asr_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nlp_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tts_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(awaken_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(read_files_asr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(tuling_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES lib/libmsc.so
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY defaultconfig
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
