cmake_minimum_required(VERSION 3.8)
project(raph_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)

find_package(gz-sim REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)


add_library(raph_gz_ackermann_plugin SHARED
  src/ackermann_to_twist.cpp
)

target_link_libraries(raph_gz_ackermann_plugin
  gz-sim::core
  gz-plugin::core
  rclcpp::rclcpp
)

ament_target_dependencies(raph_gz_ackermann_plugin
  ackermann_msgs
)

add_library(raph_gz_lidar_filter_plugin SHARED
  src/lidar_angle_filter.cpp
)

target_link_libraries(raph_gz_lidar_filter_plugin
  gz-sim::core
  gz-plugin::core
  gz-transport::core
  gz-msgs::core
)

install(
  TARGETS
    raph_gz_ackermann_plugin
    raph_gz_lidar_filter_plugin
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.sh")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv")

ament_package()

