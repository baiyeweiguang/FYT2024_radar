cmake_minimum_required(VERSION 3.10)
project(radar_referee)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(xmlrpcpp REQUIRED)
# Uncomment if the package depends on actionlib_msgs
# find_package(actionlib_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ executable
file(GLOB ALL_SOURCES "src/*.cpp" "src/common/*.cpp" "src/ui/*.cpp")
ament_auto_add_executable(${PROJECT_NAME} ${ALL_SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  serial
  sensor_msgs
  radar_interfaces
  tf2_geometry_msgs
  std_msgs
  nav_msgs
  # Uncomment if needed
  # actionlib_msgs
)

# Export dependencies to downstream packages
ament_export_dependencies(rclcpp serial sensor_msgs radar_interfaces tf2_geometry_msgs std_msgs nav_msgs)

# Uncomment if there are services or actions that need to be exported
# ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
