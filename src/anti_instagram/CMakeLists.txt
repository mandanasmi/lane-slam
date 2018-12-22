cmake_minimum_required(VERSION 2.8.3)
project(anti_instagram)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  roscpp
  rospy
)


catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/antiinstagram_correctness_test.test)
    add_rostest(tests/antiinstagram_stub_test.test)
    add_rostest(tests/antiinstagram_performance_test.test)
endif()
