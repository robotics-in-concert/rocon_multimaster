##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_unreliable_experiments)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

##############################################################################
# Project
##############################################################################

add_executable(multimaster_experimental_talker src/talker.cpp)
add_executable(multimaster_experimental_listener src/listener.cpp)
add_executable(multimaster_experimental_listener_unreliable src/listener_unreliable.cpp)

target_link_libraries(multimaster_experimental_talker ${catkin_LIBRARIES})
target_link_libraries(multimaster_experimental_listener ${catkin_LIBRARIES})
target_link_libraries(multimaster_experimental_listener_unreliable ${catkin_LIBRARIES})

set_target_properties(multimaster_experimental_talker PROPERTIES OUTPUT_NAME talker)
set_target_properties(multimaster_experimental_listener PROPERTIES OUTPUT_NAME listener)
set_target_properties(multimaster_experimental_listener_unreliable PROPERTIES OUTPUT_NAME listener_unreliable)

##############################################################################
# Installs
##############################################################################

install(TARGETS multimaster_experimental_talker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS multimaster_experimental_listener RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS multimaster_experimental_listener_unreliable RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

