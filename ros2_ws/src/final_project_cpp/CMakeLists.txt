cmake_minimum_required(VERSION 3.8)
project(final_project_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(final_project_component SHARED src/final_project.cpp)
ament_target_dependencies(final_project_component rclcpp turtlesim rclcpp_action
  geometry_msgs my_robot_interfaces rclcpp_lifecycle rclcpp_components)
rclcpp_components_register_nodes(final_project_component "final_project::TurtleController")

add_executable(final_project_step3 src/final_project_step3.cpp)
ament_target_dependencies(final_project_step3 rclcpp turtlesim rclcpp_action
    geometry_msgs my_robot_interfaces rclcpp_lifecycle)

install(TARGETS
  final_project_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
final_project_step3
  DESTINATION lib/${PROJECT_NAME}/
  )

ament_package()
