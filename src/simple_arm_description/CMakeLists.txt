cmake_minimum_required(VERSION 3.8)   # CMakeの最低バージョン指定
project(simple_arm_description)       # パッケージ名を宣言

# コンパイラが GCC または Clang の場合に警告オプションを追加
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 のビルドシステム ament_cmake を必須として読み込む
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)             # ROS2ノード作成に必要
find_package(trajectory_msgs REQUIRED)    # JointTrajectoryメッセージを使うために必要

# テスト用の設定（lintチェックなど）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権チェックをスキップ（必要ならコメントアウト）
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint をスキップ（必要ならコメントアウト）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==============================
# 実行ファイルの定義
# ==============================
# src/simple_arm_trajectory.cpp をビルド対象に追加
add_executable(simple_arm_trajectory src/simple_arm_trajectory.cpp)

# このノードが依存するパッケージを指定
ament_target_dependencies(simple_arm_trajectory rclcpp trajectory_msgs)

# ==============================
# インストール設定
# ==============================
# urdf, launch, rviz, config ディレクトリをインストール対象にする
# → colcon build 後に install/share/simple_arm_description/ 以下に配置される
# → Launchファイルから get_package_share_directory() で参照可能になる
install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.urdf"      # URDFファイル
    PATTERN "*.xacro"     # Xacroファイル
    PATTERN "*.launch.py" # Launchファイル
    PATTERN "*.rviz"      # RViz設定ファイル
    PATTERN "*.yaml"      # YAML設定ファイル
)

# ノードをインストール対象に追加
# → colcon build 後に install/lib/simple_arm_description/ に配置される
install(TARGETS
  simple_arm_trajectory
  DESTINATION lib/${PROJECT_NAME})

# ROS2 パッケージとして宣言
ament_package()