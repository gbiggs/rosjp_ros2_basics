cmake_minimum_required(VERSION 3.5)
project(displayer)

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
find_package(class_loader REQUIRED)
find_package(greeter_custom_msg REQUIRED)

include_directories(include)

# Build a shared library containing the composable node (the "component")
add_library(displayer_component SHARED src/displayer_component.cpp)
# Set the definition needed to expose the component class in the shared library
target_compile_definitions(displayer_component PRIVATE "DISPLAYER_BUILDING_DLL")
ament_target_dependencies(displayer_component
  rclcpp
  class_loader
  greeter_custom_msg
  )
# Register the component plugin with the ament resource index
rclcpp_register_node_plugins(displayer_component "displayer::Displayer")

# Build a standalone version of the node
add_executable(displayer src/displayer.cpp)
# Link in the shared library containing the component
target_link_libraries(displayer displayer_component)
ament_target_dependencies(displayer rclcpp std_msgs)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_libraries(displayer_component)

# Install the component header file
install(DIRECTORY
  include/displayer
  DESTINATION include
  )

# Install the component library
install(TARGETS
  displayer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install the standalone version of the node
install(TARGETS
  displayer
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
