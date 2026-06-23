cmake_minimum_required(VERSION 3.8)
project(test_robot_description)

find_package(ament_cmake REQUIRED)

# Install all resource directories so ROS can find them at runtime.
# 'config' is new — it holds the slam_toolbox YAML parameter file.
install(DIRECTORY launch models worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
