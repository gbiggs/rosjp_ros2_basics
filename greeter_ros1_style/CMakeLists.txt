cmake_minimum_required(VERSION 3.5)
# パッケージ名
project(greeter_ros1_style)

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
find_package(std_msgs REQUIRED)

# ノードの実行ファイルをコンパイルする
add_executable(greeter src/greeter.cpp)
# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(greeter rclcpp std_msgs)

# ノードの実行ファイルをインストールする（必須）
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

# amentのリソースインデクスにパッケージを登録する
ament_package()
