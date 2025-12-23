cmake_minimum_required(VERSION 3.22)
project(simple_arm_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

# launch フォルダがあればインストール
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

# config フォルダをインストール
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# URDF フォルダをインストール
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# setup_assistant ファイルをインストール
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})