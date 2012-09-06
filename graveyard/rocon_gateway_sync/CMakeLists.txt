##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

##############################################################################
# Ros
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(ROS_BUILD_TYPE RelWithDebInfo)

##############################################################################
# Actual project configuration
##############################################################################

#add_subdirectory(src)
