cmake_minimum_required(VERSION 2.8.3)
project(xbot_face)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
add_message_files(
  DIRECTORY msg
  FILES
  FaceResult.msg
)

generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xbot_face
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



set(OpenCV_DIR /usr/local/share/OpenCV) 
message(STATUS "    version: ${OpenCV_VERSION}") 
find_package(OpenCV REQUIRED) 
add_executable(pub_camera_image src/camera_image.cpp)
include_directories(${OpenCV_INCLUDE_DIRS}) 
target_link_libraries(pub_camera_image ${OpenCV_LIBS})
target_link_libraries(pub_camera_image ${catkin_LIBRARIES})

add_executable(wenhou src/wenhou.cpp)
target_link_libraries(wenhou ${catkin_LIBRARIES})
add_dependencies(wenhou xbot_face_gencpp)

install(TARGETS pub_camera_image
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS wenhou
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS script/face_recog.py script/face_register.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
