cmake_minimum_required(VERSION 3.5)
project(greeter_ros2_style)

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
find_package(std_msgs REQUIRED)
find_package(class_loader REQUIRED) # Needed to make loading the component dynamically easy

include_directories(include)

# Build a shared library containing the composable node (the "component")
add_library(greeter_r2s_component SHARED src/greeter_component.cpp)
# Set the definition needed to expose the component class in the shared library
target_compile_definitions(greeter_r2s_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_r2s_component
  rclcpp
  std_msgs
  class_loader
  )
# Register the component plugin with the ament resource index
rclcpp_register_node_plugins(greeter_r2s_component "greeter_ros2_style::Greeter")

# Build a standalone version of the node
add_executable(greeter src/greeter.cpp)
# Link in the shared library containing the component
target_link_libraries(greeter greeter_r2s_component)
ament_target_dependencies(greeter rclcpp std_msgs)

# Install the component header file
install(DIRECTORY
  include/greeter_ros2_style
  DESTINATION include
  )

# Install the component library
install(TARGETS
  greeter_r2s_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# Install the standalone version of the node
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
