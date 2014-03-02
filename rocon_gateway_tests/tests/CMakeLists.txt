##############################################################################
# Tests
##############################################################################
#
# This is only run when CATKIN_ENABLE_TESTING is true.

# Unit tests not needing a running ROS core.
catkin_add_nosetests(nose)

# Unit tests using nose, but needing a running ROS core.
#add_rostest(ros/utilities.test)

# Unit tests running on multimaster using rocon_test 
add_subdirectory(rocon)
