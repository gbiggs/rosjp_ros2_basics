cmake_minimum_required(VERSION 3.5)
project(displayer_basic_version)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(displayer_component SHARED src/displayer_component.cpp)
target_compile_definitions(displayer_component PRIVATE "DISPLAYER_BUILDING_DLL")
ament_target_dependencies(displayer_component
  rclcpp
  std_msgs
  rclcpp_components
  )
rclcpp_components_register_nodes(displayer_component "displayer_basic_version::Displayer")

add_executable(displayer src/displayer.cpp)
target_link_libraries(displayer displayer_component)
ament_target_dependencies(displayer rclcpp std_msgs)

install(DIRECTORY
  include/displayer
  DESTINATION include
  )

install(TARGETS
  displayer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  displayer
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
