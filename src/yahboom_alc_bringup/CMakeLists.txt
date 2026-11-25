cmake_minimum_required(VERSION 3.8)
project(yahboom_alc_bringup)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pipe)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wno-psabi)
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(nlohmann_json REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(yahboom_alc_bringup_lib
  src/explorer/backtrace_recovery.cpp
  src/explorer/exploration_memory.cpp
  src/explorer/frontier_search.cpp
  src/explorer/grid_planning.cpp
  src/diagnostics/planner_event.cpp
  src/diagnostics/planner_runtime_logger.cpp
  src/driver/r2_cmd_vel_bridge.cpp
  src/driver/r2_stall_detector_core.cpp
  src/driver/recent_free_costmap_history.cpp
  src/explorer/recovery_state_machine.cpp
  src/navigation/straight_tail.cpp
)
set_target_properties(yahboom_alc_bringup_lib PROPERTIES UNITY_BUILD ON)
target_compile_features(yahboom_alc_bringup_lib PUBLIC cxx_std_20)
target_include_directories(yahboom_alc_bringup_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(yahboom_alc_bringup_lib
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(yahboom_alc_bringup_lib
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# The explorer node class, shared by its executable and the sequence tests.
add_library(cluster_frontier_explorer_node STATIC
  src/explorer/cluster_frontier_explorer_node.cpp
  src/explorer/observation_turn.cpp
  src/explorer/recovery_driver.cpp
  src/explorer/route_executor.cpp
)
target_include_directories(cluster_frontier_explorer_node
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(cluster_frontier_explorer_node
  yahboom_alc_bringup_lib
)
ament_target_dependencies(cluster_frontier_explorer_node
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

add_executable(cluster_frontier_explorer
  src/explorer/explorer_main.cpp
)
target_link_libraries(cluster_frontier_explorer
  cluster_frontier_explorer_node
)
ament_target_dependencies(cluster_frontier_explorer
  geometry_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_ros
  visualization_msgs
)

add_executable(planner_runtime_logger
  src/diagnostics/planner_runtime_logger_node.cpp
)
target_link_libraries(planner_runtime_logger
  yahboom_alc_bringup_lib
)
ament_target_dependencies(planner_runtime_logger
  action_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2_ros
)

add_executable(r2_safety_stop
  src/driver/r2_safety_stop_node.cpp
)
target_link_libraries(r2_safety_stop
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_safety_stop
  geometry_msgs
  rclcpp
)

add_executable(r2_safe_driver
  src/driver/r2_safe_driver_supervisor.cpp
)
target_link_libraries(r2_safe_driver
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_safe_driver
  ament_index_cpp
  geometry_msgs
  rclcpp
)

add_executable(yahboom_r2_cmd_vel_bridge
  src/driver/yahboom_r2_cmd_vel_bridge_main.cpp
)
target_link_libraries(yahboom_r2_cmd_vel_bridge
  yahboom_alc_bringup_lib
)
ament_target_dependencies(yahboom_r2_cmd_vel_bridge
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)

add_executable(r2_stall_detector
  src/driver/r2_stall_detector.cpp
)

add_executable(r2_planner_server
  src/navigation/r2_planner_server.cpp
)
ament_target_dependencies(r2_planner_server
  nav2_planner
  rclcpp
)

add_library(straight_tail_planner SHARED
  src/navigation/straight_tail.cpp
  src/navigation/straight_tail_planner.cpp
)
target_compile_features(straight_tail_planner PUBLIC cxx_std_20)
target_include_directories(straight_tail_planner
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(straight_tail_planner
  nav2_core
  nav2_costmap_2d
  pluginlib
  rclcpp
  tf2
)
pluginlib_export_plugin_description_file(nav2_core straight_tail_planner_plugin.xml)

add_executable(depth_cloud_preprocessor
  src/perception/depth_cloud_preprocessor.cpp
)
target_include_directories(depth_cloud_preprocessor
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(depth_cloud_preprocessor
  ${PCL_LIBRARIES}
)
ament_target_dependencies(depth_cloud_preprocessor
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(rtabmap_map_tf_relay
  src/perception/rtabmap_map_tf_relay.cpp
)
ament_target_dependencies(rtabmap_map_tf_relay
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(r2_stall_detector
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_stall_detector
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    yahboom_alc_bringup_lib
    cluster_frontier_explorer
    depth_cloud_preprocessor
    planner_runtime_logger
    r2_safe_driver
    r2_safety_stop
    r2_stall_detector
    r2_planner_server
    straight_tail_planner
    rtabmap_map_tf_relay
    yahboom_r2_cmd_vel_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_r2_cmd_vel_bridge
    test/test_r2_cmd_vel_bridge.cpp
  )
  target_link_libraries(test_r2_cmd_vel_bridge
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_r2_cmd_vel_bridge
    nav_msgs
  )

  ament_add_gtest(test_cluster_frontier_explorer
    test/test_cluster_frontier_explorer.cpp
  )
  target_link_libraries(test_cluster_frontier_explorer
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_cluster_frontier_explorer
    nav_msgs
  )

  ament_add_gtest(test_straight_tail
    test/test_straight_tail.cpp
  )
  target_link_libraries(test_straight_tail
    yahboom_alc_bringup_lib
  )

  ament_add_gtest(test_explorer_node_sequences
    test/test_explorer_node_sequences.cpp
    TIMEOUT 300
  )
  target_link_libraries(test_explorer_node_sequences
    cluster_frontier_explorer_node
  )

  ament_add_gtest(test_backtrace_recovery
    test/test_backtrace_recovery.cpp
  )
  target_link_libraries(test_backtrace_recovery
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_backtrace_recovery
    nav_msgs
  )

  ament_add_gtest(test_recovery_state_machine
    test/test_recovery_state_machine.cpp
  )
  target_link_libraries(test_recovery_state_machine
    yahboom_alc_bringup_lib
  )

  ament_add_gtest(test_planner_event
    test/test_planner_event.cpp
  )
  target_link_libraries(test_planner_event
    yahboom_alc_bringup_lib
  )

  ament_add_gtest(test_planner_runtime_logger
    test/test_planner_runtime_logger.cpp
  )
  target_link_libraries(test_planner_runtime_logger
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_planner_runtime_logger
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_r2_stall_detector
    test/test_r2_stall_detector.cpp
  )

  target_link_libraries(test_r2_stall_detector
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_r2_stall_detector
    geometry_msgs
    nav_msgs
    rclcpp
    sensor_msgs
  )

  # Humble's GoogleTest relies on a transitive cstdint include absent in newer GCC.
  if(TARGET gtest AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    target_compile_options(gtest PRIVATE -include cstdint)
  endif()
endif()

ament_package()
