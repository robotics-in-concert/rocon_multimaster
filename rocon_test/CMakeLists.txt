##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_test)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package(
  CFG_EXTRAS rocon_test-extras.cmake
)

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARED_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARED_DESTINATION})
install(
    PROGRAMS 
        scripts/listener.py
        scripts/talker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
