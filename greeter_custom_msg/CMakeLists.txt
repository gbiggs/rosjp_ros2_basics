cmake_minimum_required(VERSION 3.5)
project(greeter_custom_msg)

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
find_package(rosidl_cmake REQUIRED) # Needed for managing message generation
find_package(rosidl_default_generators REQUIRED) # Needed for generating messages

include_directories(include)

# Generate IDL and types from the message definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Greeting.msg"
  )

# Build a shared library containing the composable node
add_library(greeter_cm_component SHARED src/greeter_component.cpp)
target_compile_definitions(greeter_cm_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_cm_component
  rclcpp
  class_loader
  rosidl_default_runtime
  )
# Add a dependency on the locally-defined messages
rosidl_target_interfaces(greeter_cm_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_register_node_plugins(greeter_cm_component "greeter_custom_msg::Greeter")

# Build a standalone version of the node
add_executable(greeter src/greeter.cpp)
target_link_libraries(greeter greeter_cm_component)
ament_target_dependencies(greeter rclcpp rosidl_default_runtime)
# Add a dependency on the locally-defined messages
rosidl_target_interfaces(greeter ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_libraries(greeter_cm_component)

# Install the component header file
install(DIRECTORY
  include/greeter_custom_msg
  DESTINATION include
  )

# Install the component library
install(TARGETS
  greeter_cm_component
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
