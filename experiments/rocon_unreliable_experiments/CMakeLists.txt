##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_unreliable_experiments)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

##############################################################################
# Project
##############################################################################

add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(listener_unreliable src/listener_unreliable.cpp)

target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(listener_unreliable ${catkin_LIBRARIES})

##############################################################################
# Installs
##############################################################################

install(TARGETS talker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS listener RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS listener_unreliable RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

