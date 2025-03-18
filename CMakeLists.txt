cmake_minimum_required(VERSION 3.0.2)
project(climbingrobot_hardware_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)
find_package(Eigen3 3.3 REQUIRED)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_executable(ros_control_node src/ros_hw.cpp)
target_link_libraries(ros_control_node ${catkin_LIBRARIES})
install(TARGETS ros_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
