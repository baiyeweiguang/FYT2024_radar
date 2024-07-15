cmake_minimum_required(VERSION 3.8)
project(radar_calibrator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-O2)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
ament_auto_find_build_dependencies()
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(radar_interfaces REQUIRED)
# find_package(message_filters REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
)

ament_auto_add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/calibrator_node.cpp
  src/lru_decider.cpp
  src/image_widget.cpp
  src/map_widget.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

# ROS 2和Qt的包含目录
# ament_auto_target_dependencies(${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  radar_interfaces
  cv_bridge
  message_filters
  image_transport
)

# 自动处理Qt的MOC过程
set_target_properties(${PROJECT_NAME}_node PROPERTIES AUTOMOC TRUE)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
