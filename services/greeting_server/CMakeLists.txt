cmake_minimum_required(VERSION 3.5)
project(greeting_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(request_greeting_service REQUIRED)

include_directories(include)

add_library(greeting_server_component SHARED src/greeting_server_component.cpp)
target_compile_definitions(greeting_server_component PRIVATE "GREETING_SERVER_BUILDING_DLL")
ament_target_dependencies(greeting_server_component
  rclcpp
  rclcpp_components
  request_greeting_service
  )
rclcpp_components_register_nodes(greeting_server_component "greeting_server::GreetingServer")

add_executable(greeting_server src/greeting_server.cpp)
target_link_libraries(greeting_server greeting_server_component)
ament_target_dependencies(greeting_server rclcpp request_greeting_service)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(request_greeting_service)

ament_export_include_directories(include)

ament_export_libraries(greeting_server_component)

install(DIRECTORY
  include/greeting_server
  DESTINATION include
  )

install(TARGETS
  greeting_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  greeting_server
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
