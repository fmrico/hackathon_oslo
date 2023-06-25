cmake_minimum_required(VERSION 3.8)
project(bombs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bombs_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  visualization_msgs
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  bombs_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/Bomb.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
rclcpp_components_register_nodes(${PROJECT_NAME} "bombs::Bomb")

add_executable(bombs_spawn src/bombs_spawn.cpp)
ament_target_dependencies(bombs_spawn ${dependencies})
target_link_libraries(bombs_spawn ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  bombs_spawn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES meshes/bomb.dae
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
