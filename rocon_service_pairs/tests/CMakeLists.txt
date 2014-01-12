### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python

# Unit tests not needing a running ROS core.
# catkin_add_nosetests(test_common.py)

# Unit tests using nose, but needing a running ROS core.
find_package(catkin REQUIRED COMPONENTS rostest)
add_rostest(service_pairs/service_pairs.test)
add_rostest(service_pair_timeouts/service_pair_timeouts.test)