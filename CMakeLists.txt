cmake_minimum_required(VERSION 2.8.3)
project(imu_bridge_sbg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sbg_driver
  sensor_msgs
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(sbg2imu src/sbg2imu.cpp)
target_link_libraries(sbg2imu ${catkin_LIBRARIES})
