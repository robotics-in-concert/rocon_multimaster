##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_gateway_tests)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rocon_test)
catkin_package()

catkin_python_setup()

##############################################################################
# Tests
##############################################################################

add_rocon_test(test/multimaster/connect_hub_by_service.test)
add_rocon_test(test/multimaster/flips.test)
add_rocon_test(test/multimaster/pulls.test)
add_rocon_test(test/multimaster/advertisements.test)
add_rocon_test(test/multimaster/graph.test)

catkin_add_nosetests(test/unit/test_connection_cache.py)
