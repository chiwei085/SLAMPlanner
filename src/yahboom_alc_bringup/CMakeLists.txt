cmake_minimum_required(VERSION 3.8)
project(yahboom_alc_bringup)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(yahboom_alc_bringup_lib
  src/r2_cmd_vel_bridge.cpp
)
target_include_directories(yahboom_alc_bringup_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(yahboom_alc_bringup_lib
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
  rclcpp
)

install(
  TARGETS yahboom_alc_bringup_lib yahboom_r2_cmd_vel_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_r2_cmd_vel_bridge
    test/test_r2_cmd_vel_bridge.cpp
  )
  target_link_libraries(test_r2_cmd_vel_bridge
    yahboom_alc_bringup_lib
  )
endif()

ament_package()
