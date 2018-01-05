cmake_minimum_required(VERSION 3.5)
project(greeting_service)

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
find_package(rosidl_cmake REQUIRED) # Needed for managing message generation
find_package(rosidl_default_generators REQUIRED) # Needed for generating messages

# Generate IDL and types from the service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestGreeting.srv"
  )

add_executable(greetingservice src/greeting_service.cpp)
ament_target_dependencies(greetingservice rclcpp )
rosidl_target_interfaces(greetingservice ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the standalone version of the node
install(TARGETS
  greetingservice
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
