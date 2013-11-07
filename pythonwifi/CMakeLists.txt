cmake_minimum_required(VERSION 2.8.3)
project(pythonwifi)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(PROGRAMS
  examples/iwconfig.py
  examples/iwlist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  api-2011.txt
  README
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

