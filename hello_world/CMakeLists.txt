cmake_minimum_required(VERSION 3.8)
project(hello_world)

set(CMAKE_CXX_STANDARD 17)

# 依存パッケージ
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hello_world_msgs REQUIRED)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "hello_world_msgs")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)
custom_executable(talker_with_service)
custom_executable(talker_with_service_param)

# listenerノードのビルド設定
custom_executable(listener)

# client_asyncノードのビルド設定
custom_executable(client_async)

# talker/listenerノードのプロセス間通信のビルド設定
custom_executable(talker_listener_composition)

# パラメータ取得・設定ノードのビルド設定
custom_executable(set_and_get_parameters)

include_directories(include)

# 共有ライブラリの生成
#ament_target_dependencies()

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()