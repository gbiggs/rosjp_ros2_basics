cmake_minimum_required(VERSION 3.5)
project(greet_me)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(greeting_actions REQUIRED)

add_executable(greet_me src/greet_me.cpp)
ament_target_dependencies(greet_me rclcpp rclcpp_action greeting_actions)

install(TARGETS
  greet_me
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
