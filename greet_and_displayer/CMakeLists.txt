cmake_minimum_required(VERSION 3.5)
project(greet_and_displayer)

# Must use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Display all warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the packages we need to build this node
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(greeter_custom_msg REQUIRED)
find_package(displayer REQUIRED)

# Build a node that composes the other two nodes
add_executable(greet_and_displayer src/greet_and_displayer.cpp)
ament_target_dependencies(greet_and_displayer
  rclcpp
  greeter_custom_msg
  displayer
  )

# Install the node
install(TARGETS
  greet_and_displayer
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
