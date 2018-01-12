cmake_minimum_required(VERSION 3.5)
# パッケージ名
project(greeter_ros2_style)

# C++14を利用する
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# すべてのワーニングを表示する
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存するパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# コンポーネントノードの共有ライブラリをコンパイルする
add_library(greeter_component SHARED src/greeter_component.cpp)
# コンポーネントノードのヘッダーファイルのマクロを設定する
target_compile_definitions(greeter_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_component
  rclcpp
  std_msgs
  class_loader
  )
# コンポーネントノードをamentのリソースインデクスに登録する
rclcpp_register_node_plugins(greeter_component "greeter_ros2_style::Greeter")

# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(greeter src/greeter.cpp)
# 共有ライブラリに実行ファイルをリンクする
target_link_libraries(greeter greeter_component)
# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(greeter rclcpp std_msgs)

# 本パッケージが利用するパッケージの情報を他のパッケージにエクスポートする
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(class_loader)
ament_export_dependencies(std_msgs)

ament_export_include_directories(include)

# 本パッケージが提供する再利用可能なコンパイルターゲットの情報をエクスポートする
ament_export_libraries(displayer_component)

# コンポーネントノードのヘッダーファイルをインストールする
install(DIRECTORY
  include/greeter_ros2_style
  DESTINATION include
  )

# コンポーネントノードの共有ライブラリをインストールする
install(TARGETS
  greeter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# スタンドアローンの実行ファイルをインストールする
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

# amentのリソースインデクスにパッケージを登録する
ament_package()
