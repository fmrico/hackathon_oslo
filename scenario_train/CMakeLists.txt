cmake_minimum_required(VERSION 3.8)
project(scenario_train)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bombs_msgs REQUIRED)
find_package(bombs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  bombs_msgs
  bombs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}/Artificier.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
rclcpp_components_register_nodes(${PROJECT_NAME} "scenario_train::Artificier")

add_executable(bombs_spawn src/bombs_spawn.cpp)
ament_target_dependencies(bombs_spawn ${dependencies})

add_executable(artificier src/artificier.cpp)
ament_target_dependencies(artificier ${dependencies})
target_link_libraries(artificier ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  bombs_spawn
  artificier
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
