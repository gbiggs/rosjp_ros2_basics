cmake_minimum_required(VERSION 3.5)
project(greeting_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(request_greeting_service REQUIRED)

add_executable(greeting_client src/greeting_client.cpp)
ament_target_dependencies(greeting_client rclcpp request_greeting_service)

install(TARGETS
  greeting_client
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
