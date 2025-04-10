cmake_minimum_required(VERSION 3.8)
project(custom_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(half_way_spin_node src/half_way_spin_node.cpp)

ament_target_dependencies(half_way_spin_node rclcpp nav2_msgs nav_msgs geometry_msgs lifecycle_msgs rclcpp_action)

add_executable(half_way_spin_bt_node src/half_way_spin_bt_node.cpp)

ament_target_dependencies(half_way_spin_bt_node rclcpp nav2_msgs nav_msgs geometry_msgs lifecycle_msgs rclcpp_action)

install(TARGETS
  half_way_spin_node half_way_spin_bt_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  src
  DESTINATION share/${PROJECT_NAME})

ament_package()
