cmake_minimum_required(VERSION 3.5)
project(greeter_custom_msg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(greeting_msg REQUIRED)

include_directories(include)

add_library(greeter_component SHARED src/greeter_component.cpp)
target_compile_definitions(greeter_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_component
  rclcpp
  rclcpp_components
  greeting_msg
  )
rclcpp_components_register_nodes(greeter_component "greeter_custom_msg::Greeter")

add_executable(greeter src/greeter.cpp)
target_link_libraries(greeter greeter_component)
ament_target_dependencies(greeter rclcpp rclcpp_components greeting_msg)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(greeting_msg)

ament_export_include_directories(include)

ament_export_libraries(greeter_component)

install(DIRECTORY
  include/greeter
  DESTINATION include
  )

install(TARGETS
  greeter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
