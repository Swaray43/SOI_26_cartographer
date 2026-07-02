cmake_minimum_required(VERSION 3.8)
project(test_robot_description)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/lidar_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_tf_publisher
)

install(DIRECTORY launch models worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
