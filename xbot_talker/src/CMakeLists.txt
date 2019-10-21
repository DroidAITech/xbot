cmake_minimum_required(VERSION 2.4.6)

file(GLOB SOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)


#aux_source_directory(. SOURCE)

add_compile_options(-std=c++11)


include_directories(include ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/)

link_directories(/usr/local/lib ${PROJECT_SOURCE_DIR}/lib)


add_executable(talker ${SOURCE})

target_link_libraries(talker aiui msc asound pthread  ${catkin_LIBRARIES})

#install(TARGETS talker  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
