cmake_minimum_required(VERSION 3.8)
project(yahboom_alc_bringup)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
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
find_package(nav2_msgs REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(yahboom_alc_bringup_lib
  src/backtrace_recovery.cpp
  src/frontier_explorer_core.cpp
  src/planner_event.cpp
  src/planner_runtime_logger.cpp
  src/r2_collision_safety_core.cpp
  src/r2_cmd_vel_bridge.cpp
  src/r2_stall_detector_core.cpp
  src/recovery_state_machine.cpp
)
set_target_properties(yahboom_alc_bringup_lib PROPERTIES UNITY_BUILD ON)
target_include_directories(yahboom_alc_bringup_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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

add_executable(cluster_frontier_explorer
  src/cluster_frontier_explorer_node.cpp
)
target_link_libraries(cluster_frontier_explorer
  yahboom_alc_bringup_lib
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
  src/planner_runtime_logger_node.cpp
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
  src/r2_safety_stop_node.cpp
)
target_link_libraries(r2_safety_stop
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_safety_stop
  geometry_msgs
  rclcpp
)

add_executable(r2_emergency_stop
  src/r2_emergency_stop_node.cpp
)
target_link_libraries(r2_emergency_stop
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_emergency_stop
  geometry_msgs
  rclcpp
)

add_executable(r2_safe_driver
  src/r2_safe_driver_supervisor.cpp
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
  src/yahboom_r2_cmd_vel_bridge_main.cpp
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
  src/r2_stall_detector.cpp
)

add_executable(r2_planner_server
  src/r2_planner_server.cpp
)
ament_target_dependencies(r2_planner_server
  nav2_planner
  rclcpp
)

add_executable(rtabmap_map_tf_relay
  src/rtabmap_map_tf_relay.cpp
)
ament_target_dependencies(rtabmap_map_tf_relay
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(r2_collision_safety_gate
  src/r2_collision_safety.cpp
)
target_link_libraries(r2_collision_safety_gate
  yahboom_alc_bringup_lib
)
ament_target_dependencies(r2_collision_safety_gate
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
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
    planner_runtime_logger
    r2_emergency_stop
    r2_safe_driver
    r2_safety_stop
    r2_stall_detector
    r2_collision_safety_gate
    r2_planner_server
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

  ament_add_gtest(test_r2_collision_safety
    test/test_r2_collision_safety.cpp
  )
  target_link_libraries(test_r2_collision_safety
    yahboom_alc_bringup_lib
  )
  ament_target_dependencies(test_r2_collision_safety
    geometry_msgs
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

  ament_add_gtest(test_rtabmap_map_tf_relay
    test/test_rtabmap_map_tf_relay.cpp
  )
  ament_target_dependencies(test_rtabmap_map_tf_relay
    tf2
  )

endif()

ament_package()
