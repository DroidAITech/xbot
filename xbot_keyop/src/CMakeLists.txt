##############################################################################
# Sources
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(xbot_keyop ${SOURCES})
add_dependencies(xbot_keyop xbot_msgs_gencpp)
target_link_libraries(xbot_keyop ${catkin_LIBRARIES})

install(TARGETS xbot_keyop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
