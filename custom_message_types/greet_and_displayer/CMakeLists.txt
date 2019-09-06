cmake_minimum_required(VERSION 3.5)
project(greet_and_displayer_custom_msg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(greeter_custom_msg REQUIRED)
find_package(displayer_custom_msg REQUIRED)

# 他のノードを統合するノードをコンパイルする
add_executable(greet_and_displayer src/greet_and_displayer.cpp)
ament_target_dependencies(greet_and_displayer
  rclcpp
  greeter_custom_msg
  displayer_custom_msg
  )

install(TARGETS
  greet_and_displayer
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
