cmake_minimum_required(VERSION 3.8)
project(test_robot_description)

find_package(ament_cmake REQUIRED)

# Install the scan relay Python script as an executable.
# install(PROGRAMS) creates lib/test_robot_description/ automatically,
# which is exactly what ROS2 looks for when you run a node from this package.
# The script must have a shebang line (#!/usr/bin/env python3).
install(PROGRAMS
  scripts/scan_relay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_relay_node
)

# Install all resource directories
install(DIRECTORY launch models worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
