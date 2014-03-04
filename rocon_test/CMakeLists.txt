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
catkin_add_env_hooks(15.rocon_test SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

##############################################################################
# Installs
##############################################################################

install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY shells DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
    PROGRAMS 
        scripts/listener.py
        scripts/talker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
