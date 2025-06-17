cmake_minimum_required(VERSION 3.10)
project(sensors_hw VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/filter_pid.py
  nodes/filter_sensor_filtering.py
  nodes/movement_detection_pid.py
  nodes/movement_detection_sensor_filtering.py
  nodes/ros_lidar_1.py
  nodes/ros_lidar_2.py
  nodes/fake_scan_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
