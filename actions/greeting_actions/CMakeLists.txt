cmake_minimum_required(VERSION 3.5)
project(greeting_actions)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# アクション定義からヘッダーファイル等を生成する
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ProcessGreeting.action"
  DEPENDENCIES action_msgs
  )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
