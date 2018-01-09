cmake_minimum_required(VERSION 3.5)
project(managed_greeter)

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
find_package(class_loader REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_library(managed_greeter_component SHARED src/managed_greeter_component.cpp)
# Set the definition needed to expose the component class in the shared library
target_compile_definitions(managed_greeter_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(managed_greeter_component
  rclcpp
  greeter_custom_msg
  class_loader
  lifecycle_msgs
  rclcpp_lifecycle
  )
# Register the component plugin with the ament resource index
rclcpp_register_node_plugins(managed_greeter_component "managed_greeter::Greeter")

add_executable(managed_greeter src/managed_greeter.cpp)
target_link_libraries(managed_greeter managed_greeter_component)
ament_target_dependencies(managed_greeter
  rclcpp
  greeter_custom_msg
  lifecycle_msgs
  rclcpp_lifecycle
  )

# Install the component header file
install(DIRECTORY
  include/managed_greeter
  DESTINATION include
  )

# Install the component library
install(TARGETS
  managed_greeter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install the standalone version of the node
install(TARGETS
  managed_greeter
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
