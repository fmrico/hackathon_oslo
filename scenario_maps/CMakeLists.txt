cmake_minimum_required(VERSION 3.5)
project(scenario_maps)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bombs_msgs REQUIRED)
find_package(bombs REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    ament_index_cpp
    bombs_msgs
    bombs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_executable(bombs_spawn_house src/bombs_spawn_house.cpp)
ament_target_dependencies(bombs_spawn_house ${dependencies})

add_executable(bombs_spawn_bookstore src/bombs_spawn_bookstore.cpp)
ament_target_dependencies(bombs_spawn_bookstore ${dependencies})

add_executable(bombs_spawn_warehouse src/bombs_spawn_warehouse.cpp)
ament_target_dependencies(bombs_spawn_warehouse ${dependencies})

add_executable(bombs_spawn_hospital src/bombs_spawn_hospital.cpp)
ament_target_dependencies(bombs_spawn_hospital ${dependencies})


add_library(approach_bomb_bt_node SHARED src/scenario_maps/ApproachBomb.cpp)
add_library(deactivate_bomb_bt_node SHARED src/scenario_maps/DeactivateBomb.cpp)
add_library(is_bomb_detected_bt_node SHARED src/scenario_maps/IsBombDetected.cpp)
add_library(is_bomb_near_bt_node SHARED src/scenario_maps/IsBombNear.cpp)
add_library(look_for_bomb_bt_node SHARED src/scenario_maps/LookForBomb.cpp)
add_library(move_bt_node SHARED src/scenario_maps/Move.cpp)

list(APPEND plugin_libs
  approach_bomb_bt_node
  deactivate_bomb_bt_node
  is_bomb_detected_bt_node
  is_bomb_near_bt_node
  look_for_bomb_bt_node
  move_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(artificier src/artificier.cpp)
ament_target_dependencies(artificier ${dependencies})
target_link_libraries(artificier ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  artificier
  bombs_spawn_house
  bombs_spawn_bookstore
  bombs_spawn_warehouse
  bombs_spawn_hospital
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config behavior_tree_xml launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
