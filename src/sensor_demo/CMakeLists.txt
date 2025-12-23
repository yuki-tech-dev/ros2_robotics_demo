cmake_minimum_required(VERSION 3.8)
project(sensor_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(laser_publisher src/laser_publisher.cpp)
ament_target_dependencies(laser_publisher rclcpp sensor_msgs)

#CMakeLists.txt で install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME}/) と書くことで、
#ビルド後に install/sensor_demo/share/sensor_demo/urdf/ にコピーされます。
#そのため、launch ファイルからは get_package_share_directory('sensor_demo') を使って正しく参照できるようになります。
install(TARGETS
  laser_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()