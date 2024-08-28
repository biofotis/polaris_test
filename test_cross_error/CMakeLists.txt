cmake_minimum_required(VERSION 3.0.2)
project(test_cross_error)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rostest
  roslaunch
)
catkin_package(
  CATKIN_DEPENDS std_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/test_cross_error.test)
endif()

install(PROGRAMS
  scripts/pure_pursuit_sim_mod.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
