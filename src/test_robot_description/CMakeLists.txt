cmake_minimum_required(VERSION 3.8)
project(test_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
