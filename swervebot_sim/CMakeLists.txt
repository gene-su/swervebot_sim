cmake_minimum_required(VERSION 3.8)
project(swervebot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add swerve controller executable
add_executable(swervebot_driver src/swervebot_driver.cpp)
ament_target_dependencies(swervebot_driver 
  rclcpp 
  geometry_msgs 
  std_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Install executables
install(TARGETS
  swervebot_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install model files
install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
