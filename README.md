# ros2_basics_advances_concepts

https://udemy.com/course/ros2-advanced-core-concepts/learn/lecture/40028720#overview

# ROS2 Actions

## Why

Some features:

- Can cancel the actual executions.
- Can get feedback from the action server.
- Can handle Multiple request
- Longer actions

![[Pasted image 20231108164000.png]]

To use actions, we need first to create a package where all the actions/services/topics live. To do that, we need to create a C++ package.

When the C++ is created we proceed to create 3 directories:

- action
- srv
- msg

and modify the CMakeList.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

```

also the packages.xml

```xml
...
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
...
```

## Python
