##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_gateway)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED
  COMPONENTS roslint)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

# We double up on gateway_info/remote_gateway_info - both private (backwards
# compatible) and public in setup.py
install(
    PROGRAMS 
        scripts/gateway_info
        scripts/remote_gateway_info
        scripts/gateway.py
        scripts/advertise
        scripts/flip
        scripts/pull
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
     RELATIVE ${PROJECT_SOURCE_DIR} scripts/*)
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRC
RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})
roslint_python(${${PROJECT_NAME}_PY_SRC})

