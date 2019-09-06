cmake_minimum_required(VERSION 3.5)
project(greeting_processor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(greeting_actions REQUIRED)

include_directories(include)

add_library(greeting_processor_component SHARED src/greeting_processor_component.cpp)
target_compile_definitions(greeting_processor_component PRIVATE "GREETING_PROCESSOR_BUILDING_DLL")
ament_target_dependencies(greeting_processor_component
  rclcpp
  rclcpp_action
  rclcpp_components
  greeting_actions
  )
rclcpp_components_register_nodes(greeting_processor_component "greeting_processor::GreetingProcessor")

add_executable(greeting_processor src/greeting_processor.cpp)
target_link_libraries(greeting_processor greeting_processor_component)
ament_target_dependencies(greeting_processor rclcpp rclcpp_action greeting_actions)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(greeting_actions)

ament_export_include_directories(include)

ament_export_libraries(greeting_processor_component)

install(DIRECTORY
  include/greeting_processor
  DESTINATION include
  )

install(TARGETS
  greeting_processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  greeting_processor
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
