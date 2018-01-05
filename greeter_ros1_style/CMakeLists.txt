cmake_minimum_required(VERSION 3.5)
# Name of the package
project(greeter_ros1_style)

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
find_package(std_msgs REQUIRED)

# Add the node executable
add_executable(greeter src/greeter.cpp)
# Set dependencies on required libraries
ament_target_dependencies(greeter rclcpp std_msgs)

# Install the node so it can be executed
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

# Register the package with the ament resource index
ament_package()
