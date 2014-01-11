##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_service_pairs)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(
#    PROGRAMS 
#        scripts/concert_roles_and_apps
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

##############################################################################
# Unit Tests
##############################################################################

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()