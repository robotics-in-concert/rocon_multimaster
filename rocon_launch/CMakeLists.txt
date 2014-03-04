##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_launch)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()
catkin_add_env_hooks(15.rocon_launch SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

##############################################################################
# Installs
##############################################################################

install(DIRECTORY shells DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
