##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_utilities)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosunit rostest)
catkin_package()
catkin_python_setup()
catkin_add_env_hooks(15.roconbash SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

##############################################################################
# Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

##############################################################################
# Installs
##############################################################################

install(FILES roconbash DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/example_slave_api
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
