cmake_minimum_required(VERSION 3.5)
project(greeting_client)

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
find_package(greeting_service REQUIRED)

add_executable(greetingclient src/greeting_client.cpp)
ament_target_dependencies(greetingclient rclcpp greeting_service)

# Install the standalone version of the node
install(TARGETS
  greetingclient
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
